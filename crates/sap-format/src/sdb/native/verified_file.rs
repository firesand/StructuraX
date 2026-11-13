//! Shared native-export I/O contract for the library, API, GUI and CLI.
use super::inspect_native_sdb_bytes;
use crate::FormatError;
use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, Write};
use std::path::Path;

pub(super) fn write_new(path: &Path, expected: &[u8]) -> Result<(), FormatError> {
    write_and_verify(path, expected, |file, bytes| file.write_all(bytes))
}

fn write_and_verify(
    path: &Path,
    expected: &[u8],
    write: impl FnOnce(&mut File, &[u8]) -> std::io::Result<()>,
) -> Result<(), FormatError> {
    let mut file = OpenOptions::new()
        .read(true)
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|error| {
            std::io::Error::new(
                error.kind(),
                format!(
                    "Cannot create {} without overwriting: {error}",
                    path.display()
                ),
            )
        })?;
    let result = (|| {
        write(&mut file, expected)?;
        file.sync_all()?;
        file.rewind()?;
        let mut actual = Vec::new();
        file.read_to_end(&mut actual)?;
        if actual != expected {
            return Err(FormatError::InvalidSdb(
                "Native export readback did not match the serialized bytes".into(),
            ));
        }
        let document = inspect_native_sdb_bytes(&actual)?;
        let inventory = document.inventory();
        if inventory.parsed_bytes != inventory.stream_bytes {
            return Err(FormatError::InvalidSdb(format!(
                "Native export readback parsed only {}/{} stream bytes",
                inventory.parsed_bytes, inventory.stream_bytes
            )));
        }
        Ok(())
    })();
    drop(file);
    if result.is_err() {
        // Creation was exclusive, so a failure cannot remove a pre-existing file.
        let _ = std::fs::remove_file(path);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{pack_model_to_native_sdb, save_native_sdb_file, write_sdb_bytes};
    use sap_model::SapModel;
    use std::sync::atomic::{AtomicU64, Ordering};

    fn path() -> std::path::PathBuf {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        std::env::temp_dir().join(format!(
            "structurax-verified-export-{}-{}-{}.sdb",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            NEXT.fetch_add(1, Ordering::Relaxed),
        ))
    }
    fn fixture() -> (SapModel, Vec<u8>) {
        let mut model = SapModel::default();
        model.materials.shift_remove("4000Psi");
        model.area_sections.clear();
        let bytes = write_sdb_bytes(&pack_model_to_native_sdb(&model).unwrap()).unwrap();
        (model, bytes)
    }

    #[test]
    fn shared_native_save_reads_back_and_never_overwrites() {
        let (model, bytes) = fixture();
        let output = path();
        save_native_sdb_file(&model, &output).unwrap();
        assert_eq!(std::fs::read(&output).unwrap(), bytes);
        let imported = crate::load_model_with_report(&output).unwrap();
        assert!(imported
            .warnings
            .iter()
            .any(|warning| warning.contains("Fys uses Fy")));
        let error = save_native_sdb_file(&model, &output).unwrap_err();
        assert!(error.to_string().contains("without overwriting"));
        assert_eq!(std::fs::read(&output).unwrap(), bytes);
        std::fs::remove_file(output).unwrap();
    }

    #[test]
    fn readback_detects_silent_truncation_and_corruption_and_cleans_output() {
        let (_, bytes) = fixture();
        for corrupt in [false, true] {
            let output = path();
            let error = write_and_verify(&output, &bytes, |file, original| {
                let mut damaged = original.to_vec();
                if corrupt {
                    // Equal size and otherwise valid container: byte equality is required.
                    damaged[0] ^= 1;
                } else {
                    damaged.truncate(damaged.len() / 2);
                }
                file.write_all(&damaged)
            })
            .unwrap_err();
            assert!(error.to_string().contains("readback did not match"));
            assert!(!output.exists());
        }
    }

    #[test]
    fn failed_write_and_invalid_serialization_leave_no_successful_export() {
        let (_, bytes) = fixture();
        let output = path();
        let error = write_and_verify(&output, &bytes, |file, original| {
            file.write_all(&original[..original.len() / 2])?;
            Err(std::io::Error::other("injected write failure"))
        })
        .unwrap_err();
        assert!(error.to_string().contains("injected write failure"));
        assert!(!output.exists());
        assert!(write_new(&output, b"not an SDB container").is_err());
        assert!(!output.exists());
    }

    #[cfg(unix)]
    #[test]
    fn native_save_refuses_symlink_destination_without_touching_target() {
        let (model, _) = fixture();
        let target = path();
        let link = path();
        std::fs::write(&target, b"original target").unwrap();
        std::os::unix::fs::symlink(&target, &link).unwrap();
        assert!(save_native_sdb_file(&model, &link).is_err());
        assert_eq!(std::fs::read(&target).unwrap(), b"original target");
        std::fs::remove_file(link).unwrap();
        std::fs::remove_file(target).unwrap();
    }
}
