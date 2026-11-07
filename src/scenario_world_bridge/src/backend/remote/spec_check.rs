//! The `spec_hash` start check (roadmap 051): an ego is one car only if every part of it
//! was built from the same vehicle spec (sim_vehicle_spec). Before an ego starts, the bridge
//! collects the `spec_hash` each part loaded -- the world server's body (the rig or the
//! physics it applied), the vehicle agent's registration (the autopilot's vehicle files), the
//! installed vehicle catalog's entry for the ego's body (what SSv2 declared it from) -- and
//! refuses the ego when two disagree, naming who loaded which. A part that reports none is
//! left out: it catches a process running an old install, which the build checks cannot see.
//!
//! The catalog is sim_vehicle_spec's when installed, plus every file the bridge's
//! `vehicle_catalogs` lists (the config file's key, then the ROS parameter): a car defined in
//! another repository brings its own catalog.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use eyre::{bail, eyre, Result, WrapErr};

/// The catalog's property that carries an entry's spec_hash.
const PROPERTY: &str = "swb.spec_hash";

/// The vehicle catalog as installed beside the bridge (sim_vehicle_spec's share, found on
/// `AMENT_PREFIX_PATH`): the one SSv2 reads when it shares the bridge's workspace.
pub fn installed_catalog() -> Option<PathBuf> {
    std::env::var("AMENT_PREFIX_PATH")
        .ok()?
        .split(':')
        .map(|prefix| PathBuf::from(prefix).join("share/sim_vehicle_spec/catalogs/vehicles.xosc"))
        .find(|path| path.is_file())
}

/// The installed catalog's `{model3d: spec_hash}`; none installed or unreadable is a warning
/// and an empty map (the ego is then not checked against a catalog).
pub fn installed_hashes() -> HashMap<String, String> {
    let Some(path) = installed_catalog() else {
        tracing::warn!(
            "No vehicle catalog installed (sim_vehicle_spec): the ego's spec is not checked \
             against it"
        );
        return HashMap::new();
    };
    std::fs::read_to_string(&path)
        .map(|text| catalog_hashes(&text))
        .unwrap_or_else(|e| {
            tracing::warn!("Reading the vehicle catalog {}: {e}", path.display());
            HashMap::new()
        })
}

/// The catalogs `vehicle_catalogs` lists: the config file's key (a list of paths, each
/// relative to the file's directory unless absolute), then the ROS parameter's (comma- or
/// colon-separated, relative to the working directory).
pub fn listed_catalogs(config_file: &Path, param: Option<&str>) -> Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    if config_file.is_file() {
        let text = std::fs::read_to_string(config_file)
            .wrap_err_with(|| format!("read {}", config_file.display()))?;
        let dir = config_file.parent().unwrap_or(Path::new("."));
        out.extend(file_catalogs(&text, dir).wrap_err_with(|| config_file.display().to_string())?);
    }
    out.extend(
        param
            .unwrap_or_default()
            .split([',', ':'])
            .map(str::trim)
            .filter(|p| !p.is_empty())
            .map(PathBuf::from),
    );
    Ok(out)
}

/// The `vehicle_catalogs` key of a bridge config file's `text`, resolved against `dir`.
fn file_catalogs(text: &str, dir: &Path) -> Result<Vec<PathBuf>> {
    let value: serde_yaml::Value = serde_yaml::from_str(text).wrap_err("parse")?;
    let Some(list) = value.get("vehicle_catalogs") else {
        return Ok(Vec::new());
    };
    let items = match list {
        serde_yaml::Value::Null => return Ok(Vec::new()),
        serde_yaml::Value::Sequence(items) => items,
        _ => bail!("vehicle_catalogs must be a list of catalog paths"),
    };
    items
        .iter()
        .map(|item| {
            let path = item
                .as_str()
                .ok_or_else(|| eyre!("vehicle_catalogs: {item:?} is not a path"))?;
            Ok(dir.join(path))
        })
        .collect()
}

/// `{model3d: spec_hash}` of a listed catalog: a file that cannot be read, is not an
/// OpenSCENARIO catalog or names no vehicle with a spec_hash is an error naming it.
pub fn read_catalog(path: &Path) -> Result<HashMap<String, String>> {
    let text = std::fs::read_to_string(path)
        .wrap_err_with(|| format!("vehicle catalog {}", path.display()))?;
    if !text.contains("<OpenSCENARIO") || !text.contains("<Catalog") {
        bail!(
            "vehicle catalog {}: not an OpenSCENARIO catalog",
            path.display()
        );
    }
    let hashes = catalog_hashes(&text);
    if hashes.is_empty() {
        bail!(
            "vehicle catalog {}: no Vehicle with a model3d and a {PROPERTY} property",
            path.display()
        );
    }
    Ok(hashes)
}

/// The union of catalogs, each with the file it came from: a body two of them give
/// different spec_hashes is an error naming both files.
pub fn union(catalogs: &[(PathBuf, HashMap<String, String>)]) -> Result<HashMap<String, String>> {
    let mut out: HashMap<String, (String, &Path)> = HashMap::new();
    for (path, hashes) in catalogs {
        for (model, hash) in hashes {
            match out.get(model) {
                Some((seen, from)) if seen != hash => bail!(
                    "vehicle catalogs disagree on {model}: {} gives spec_hash {}, {} gives {}",
                    from.display(),
                    &seen[..seen.len().min(12)],
                    path.display(),
                    &hash[..hash.len().min(12)]
                ),
                Some(_) => {}
                None => {
                    out.insert(model.clone(), (hash.clone(), path));
                }
            }
        }
    }
    Ok(out.into_iter().map(|(m, (h, _))| (m, h)).collect())
}

/// The catalog the ego is checked against when `vehicle_catalogs` lists files: the installed
/// one (if any, read as [`installed_hashes`]) and every listed one. `None` when none is listed,
/// so the installed catalog is read at the first ego spawn, as without the key.
pub fn load_listed(listed: &[PathBuf]) -> Result<Option<HashMap<String, String>>> {
    if listed.is_empty() {
        return Ok(None);
    }
    let mut catalogs = Vec::new();
    if let Some(path) = installed_catalog() {
        catalogs.push((path, installed_hashes()));
    } else {
        installed_hashes();
    }
    for path in listed {
        catalogs.push((path.clone(), read_catalog(path)?));
        tracing::info!("Vehicle catalog: {}", path.display());
    }
    union(&catalogs).map(Some)
}

/// `{model3d: spec_hash}` of a vehicle catalog's entries.
pub fn catalog_hashes(text: &str) -> HashMap<String, String> {
    let mut out = HashMap::new();
    for entry in text.split("<Vehicle ").skip(1) {
        let entry = entry.split("</Vehicle>").next().unwrap_or_default();
        let head = entry.split('>').next().unwrap_or_default();
        let (Some(model), Some(hash)) = (attribute(head, "model3d"), property(entry, PROPERTY))
        else {
            continue;
        };
        out.insert(model, hash);
    }
    out
}

fn attribute(tag: &str, name: &str) -> Option<String> {
    let start = tag.find(&format!(" {name}=\""))? + name.len() + 3;
    let len = tag[start..].find('"')?;
    Some(tag[start..start + len].to_string())
}

fn property(entry: &str, name: &str) -> Option<String> {
    entry
        .split("<Property ")
        .skip(1)
        .find(|p| attribute(&format!(" {p}"), "name").as_deref() == Some(name))
        .and_then(|p| attribute(&format!(" {p}"), "value"))
}

/// Why the parts disagree, `None` when every part that reported a hash reported the same.
pub fn disagreement(parts: &[(&str, Option<&str>)]) -> Option<String> {
    let known: Vec<(&str, &str)> = parts
        .iter()
        .filter_map(|(who, hash)| hash.filter(|h| !h.is_empty()).map(|h| (*who, h)))
        .collect();
    let first = known.first()?.1;
    if known.iter().all(|(_, h)| *h == first) {
        return None;
    }
    Some(
        known
            .iter()
            .map(|(who, h)| format!("{who} {}", &h[..h.len().min(12)]))
            .collect::<Vec<_>>()
            .join(", "),
    )
}

/// One line of who reported what, for the log.
pub fn summary(parts: &[(&str, Option<&str>)]) -> String {
    parts
        .iter()
        .map(|(who, hash)| match hash.filter(|h| !h.is_empty()) {
            Some(h) => format!("{who} {}", &h[..h.len().min(12)]),
            None => format!("{who} none"),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    const CATALOG: &str = r#"<?xml version="1.0"?>
<OpenSCENARIO><Catalog name="vehicles">
    <Vehicle name="tesla_model3" vehicleCategory="car" model3d="vehicle.tesla.model3">
      <BoundingBox><Center x="1.416" y="0.0" z="0.744" /></BoundingBox>
      <Properties>
        <Property name="swb.spec_hash" value="d5d4b266c7fd1fb4" />
      </Properties>
    </Vehicle>
    <Vehicle name="box" vehicleCategory="car">
      <Properties><Property name="swb.spec_hash" value="ffff" /></Properties>
    </Vehicle>
</Catalog></OpenSCENARIO>"#;

    #[test]
    fn the_catalog_maps_each_body_to_its_spec() {
        let h = catalog_hashes(CATALOG);
        assert_eq!(h.len(), 1, "{h:?}");
        assert_eq!(h["vehicle.tesla.model3"], "d5d4b266c7fd1fb4");
    }

    #[test]
    fn the_shipped_catalog_has_the_model3() {
        let text = include_str!("../../../../sim_vehicle_spec/catalogs/vehicles.xosc");
        assert_eq!(catalog_hashes(text)["vehicle.tesla.model3"].len(), 64);
    }

    fn scratch(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("swb_spec_check_{}_{name}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn cart(hash: &str) -> String {
        format!(
            r#"<OpenSCENARIO><Catalog name="vehicles">
  <Vehicle name="outside_car" vehicleCategory="car" model3d="vehicle.example.outside">
    <Properties><Property name="swb.spec_hash" value="{hash}" /></Properties>
  </Vehicle>
</Catalog></OpenSCENARIO>"#
        )
    }

    #[test]
    fn listed_catalogs_join_the_installed_one() {
        let dir = scratch("union");
        let model3 = dir.join("model3.xosc");
        let carts = dir.join("carts.xosc");
        std::fs::write(&model3, CATALOG).unwrap();
        std::fs::write(&carts, cart("aaaa")).unwrap();
        let h = union(&[
            (model3.clone(), read_catalog(&model3).unwrap()),
            (carts.clone(), read_catalog(&carts).unwrap()),
        ])
        .unwrap();
        assert_eq!(h.len(), 2, "{h:?}");
        assert_eq!(h["vehicle.tesla.model3"], "d5d4b266c7fd1fb4");
        assert_eq!(h["vehicle.example.outside"], "aaaa");
        // The same entry twice with one hash is no conflict.
        let again = union(&[
            (carts.clone(), read_catalog(&carts).unwrap()),
            (model3.clone(), read_catalog(&carts).unwrap()),
        ]);
        assert!(again.is_ok(), "{again:?}");
    }

    #[test]
    fn a_listed_catalog_that_is_missing_or_not_a_catalog_names_its_path() {
        let dir = scratch("missing");
        let missing = dir.join("nowhere.xosc");
        let err = read_catalog(&missing).unwrap_err();
        assert!(format!("{err:#}").contains("nowhere.xosc"), "{err:#}");
        let junk = dir.join("junk.xosc");
        std::fs::write(&junk, "not: a catalog").unwrap();
        let err = read_catalog(&junk).unwrap_err();
        assert!(format!("{err:#}").contains("junk.xosc"), "{err:#}");
        let empty = dir.join("empty.xosc");
        std::fs::write(
            &empty,
            "<OpenSCENARIO><Catalog name=\"v\"></Catalog></OpenSCENARIO>",
        )
        .unwrap();
        let err = read_catalog(&empty).unwrap_err();
        assert!(format!("{err:#}").contains("empty.xosc"), "{err:#}");
        assert!(load_listed(&[missing]).is_err());
        assert_eq!(load_listed(&[]).unwrap(), None);
    }

    #[test]
    fn two_catalogs_giving_one_body_two_specs_name_both_files() {
        let dir = scratch("conflict");
        let a = dir.join("a.xosc");
        let b = dir.join("b.xosc");
        std::fs::write(&a, cart("aaaa")).unwrap();
        std::fs::write(&b, cart("bbbb")).unwrap();
        let err = union(&[
            (a.clone(), read_catalog(&a).unwrap()),
            (b.clone(), read_catalog(&b).unwrap()),
        ])
        .unwrap_err()
        .to_string();
        assert!(err.contains("vehicle.example.outside"), "{err}");
        assert!(err.contains("a.xosc") && err.contains("b.xosc"), "{err}");
    }

    #[test]
    fn vehicle_catalogs_come_from_the_file_then_the_parameter() {
        let dir = scratch("listed");
        let config = dir.join("bridge_config.yaml");
        std::fs::write(
            &config,
            "weather: \"\"\nvehicle_catalogs:\n  - carts.xosc\n  - /abs/trucks.xosc\n",
        )
        .unwrap();
        let listed =
            listed_catalogs(&config, Some("/x/one.xosc, /y/two.xosc:/z/three.xosc")).unwrap();
        assert_eq!(
            listed,
            vec![
                dir.join("carts.xosc"),
                PathBuf::from("/abs/trucks.xosc"),
                PathBuf::from("/x/one.xosc"),
                PathBuf::from("/y/two.xosc"),
                PathBuf::from("/z/three.xosc"),
            ]
        );
        // No key, no parameter: nothing listed.
        std::fs::write(&config, "weather: \"\"\n").unwrap();
        assert!(listed_catalogs(&config, Some("")).unwrap().is_empty());
        assert!(listed_catalogs(&dir.join("absent.yaml"), None)
            .unwrap()
            .is_empty());
        std::fs::write(&config, "vehicle_catalogs: carts.xosc\n").unwrap();
        assert!(listed_catalogs(&config, None).is_err());
    }

    #[test]
    fn parts_agree_unless_two_known_hashes_differ() {
        let a = "d5d4b266c7fd1fb46630";
        assert_eq!(disagreement(&[("world", Some(a)), ("agent", None)]), None);
        assert_eq!(
            disagreement(&[("world", Some(a)), ("agent", Some(a))]),
            None
        );
        assert_eq!(disagreement(&[("world", Some("")), ("agent", None)]), None);
        assert_eq!(
            disagreement(&[
                ("the world server", Some(a)),
                ("the vehicle agent", Some("0123456789abcdef")),
                ("the catalog", None),
            ])
            .as_deref(),
            Some("the world server d5d4b266c7fd, the vehicle agent 0123456789ab")
        );
        assert_eq!(
            summary(&[("world", Some(a)), ("agent", None)]),
            "world d5d4b266c7fd, agent none"
        );
    }
}
