use crate::args::Args;
// use block constants via fully-qualified names to avoid unused-import warnings
use crate::coordinate_system::cartesian::XZBBox;
use crate::element_processing::*;
use crate::ground::Ground;
use crate::osm_parser::ProcessedElement;
use crate::progress::emit_gui_progress_update;
use crate::world_editor::WorldEditor;
use colored::Colorize;
use indicatif::{ProgressBar, ProgressStyle};
use crate::batching::{split_bbox_into_tiles, partition_elements_by_tile, ResumeState, tile_bbox};
// simplified sequential runner; Rayon removed
use std::sync::{Arc, Mutex};

pub const MIN_Y: i32 = -64;

pub fn generate_world(
    elements: Vec<ProcessedElement>,
    xzbbox: XZBBox,
    ground: Ground,
    args: &Args,
) -> Result<(), String> {
    // Worker mode: process single tile read from JSON file and exit
    if args.worker {
        if let Some(tile_file) = &args.tile_file {
            let s = std::fs::read_to_string(tile_file).map_err(|e| format!("Failed to read tile file: {}", e))?;
            let tile: crate::batching::Tile = serde_json::from_str(&s).map_err(|e| format!("Failed to parse tile file: {}", e))?;

            let elems_map = partition_elements_by_tile(&elements, &vec![tile.clone()]);
            let elems = elems_map.get(&tile.id).cloned().unwrap_or_default();

            let tile_bbox_xz = tile_bbox(&tile);
            let mut editor = WorldEditor::new(&format!("{}/region", args.path), &tile_bbox_xz);
            editor.set_ground(&ground);

            // process elements (reuse logic)
            for element in &elems {
                match element {
                    ProcessedElement::Way(way) => {
                        if way.tags.contains_key("building") || way.tags.contains_key("building:part") {
                            buildings::generate_buildings(&mut editor, way, args, None);
                        } else if way.tags.contains_key("highway") {
                            highways::generate_highways(&mut editor, element, args);
                        } else if way.tags.contains_key("landuse") {
                            landuse::generate_landuse(&mut editor, way, args);
                        } else if way.tags.contains_key("natural") {
                            natural::generate_natural(&mut editor, element, args);
                        } else if way.tags.contains_key("amenity") {
                            amenities::generate_amenities(&mut editor, element, args);
                        } else if way.tags.contains_key("leisure") {
                            leisure::generate_leisure(&mut editor, way, args);
                        } else if way.tags.contains_key("barrier") {
                            barriers::generate_barriers(&mut editor, element);
                        } else if way.tags.contains_key("waterway") {
                            waterways::generate_waterways(&mut editor, way);
                        } else if way.tags.contains_key("bridge") {
                            //bridges::generate_bridges(&mut editor, way, ground_level); // TODO FIX
                        } else if way.tags.contains_key("railway") {
                            railways::generate_railways(&mut editor, way);
                        } else if way.tags.contains_key("aeroway") || way.tags.contains_key("area:aeroway") {
                            highways::generate_aeroway(&mut editor, way, args);
                        } else if way.tags.get("service") == Some(&"siding".to_string()) {
                            highways::generate_siding(&mut editor, way);
                        } else if way.tags.contains_key("man_made") {
                            man_made::generate_man_made(&mut editor, element, args);
                        }
                    }
                    ProcessedElement::Node(node) => {
                        if node.tags.contains_key("door") || node.tags.contains_key("entrance") {
                            doors::generate_doors(&mut editor, node);
                        } else if node.tags.contains_key("natural") && node.tags.get("natural") == Some(&"tree".to_string()) {
                            natural::generate_natural(&mut editor, element, args);
                        } else if node.tags.contains_key("amenity") {
                            amenities::generate_amenities(&mut editor, element, args);
                        } else if node.tags.contains_key("barrier") {
                            barriers::generate_barrier_nodes(&mut editor, node);
                        } else if node.tags.contains_key("highway") {
                            highways::generate_highways(&mut editor, element, args);
                        } else if node.tags.contains_key("tourism") {
                            tourisms::generate_tourisms(&mut editor, node);
                        } else if node.tags.contains_key("man_made") {
                            man_made::generate_man_made_nodes(&mut editor, node);
                        }
                    }
                    ProcessedElement::Relation(rel) => {
                        if rel.tags.contains_key("building") || rel.tags.contains_key("building:part") {
                            buildings::generate_building_from_relation(&mut editor, rel, args);
                        } else if rel.tags.contains_key("water") || rel.tags.get("natural") == Some(&"water".to_string()) {
                            water_areas::generate_water_areas(&mut editor, rel);
                        } else if rel.tags.contains_key("natural") {
                            natural::generate_natural_from_relation(&mut editor, rel, args);
                        } else if rel.tags.contains_key("landuse") {
                            landuse::generate_landuse_from_relation(&mut editor, rel, args);
                        } else if rel.tags.get("leisure") == Some(&"park".to_string()) {
                            leisure::generate_leisure_from_relation(&mut editor, rel, args);
                        } else if rel.tags.contains_key("man_made") {
                            man_made::generate_man_made(&mut editor, &ProcessedElement::Relation(rel.clone()), args);
                        }
                    }
                }
            }

            // generate tile ground
            for x in tile.min_x..=tile.max_x {
                for z in tile.min_z..=tile.max_z {
                    if !editor.check_for_block(x, 0, z, Some(&[crate::block_definitions::STONE])) {
                        editor.set_block(crate::block_definitions::GRASS_BLOCK, x, 0, z, None, None);
                        editor.set_block(crate::block_definitions::DIRT, x, -1, z, None, None);
                        editor.set_block(crate::block_definitions::DIRT, x, -2, z, None, None);
                    }
                    if args.fillground {
                        editor.fill_blocks_absolute(
                            crate::block_definitions::STONE,
                            x,
                            MIN_Y + 1,
                            z,
                            x,
                            editor.get_absolute_y(x, -3, z),
                            z,
                            None,
                            None,
                        );
                    }
                    editor.set_block_absolute(crate::block_definitions::BEDROCK, x, MIN_Y, z, None, Some(&[crate::block_definitions::BEDROCK]));
                }
            }

            editor.save();
            return Ok(());
        } else {
            return Err("--worker requires --tile-file".to_string());
        }
    }
    let region_dir: String = format!("{}/region", args.path);

    println!("{} Processing data...", "[4/7]".bold());

    println!("{} Processing terrain...", "[5/7]".bold());
    emit_gui_progress_update(25.0, "Processing terrain...");

    // Prepare tiles and partition elements
    let tiles = split_bbox_into_tiles(&xzbbox, args.tile_size);
    let partitions = partition_elements_by_tile(&elements, &tiles);

    // Resume handled via in-memory state below

    // Iterate tiles (parallel up to max_parallel_tiles)
    let total_tiles = tiles.len() as u64;
    let tile_pb = ProgressBar::new(total_tiles);
    tile_pb.set_style(ProgressStyle::default_bar()
        .template("{spinner:.green} [{elapsed_precise}] [{bar:45.white/black}] {pos}/{len} tiles ({eta}) {msg}")
        .unwrap()
        .progress_chars("█▓░"));

    // Build region lock map to avoid concurrent writes to the same region files
    let mut region_locks_map: std::collections::HashMap<(i32, i32), Arc<Mutex<()>>> = std::collections::HashMap::new();
    for tile in &tiles {
        let rx_min = tile.min_x >> 9; // chunk >>4 then region >>5 => >>9
        let rx_max = tile.max_x >> 9;
        let rz_min = tile.min_z >> 9;
        let rz_max = tile.max_z >> 9;
        for rx in rx_min..=rx_max {
            for rz in rz_min..=rz_max {
                region_locks_map.entry((rx, rz)).or_insert_with(|| Arc::new(Mutex::new(())));
            }
        }
    }
    let region_locks = Arc::new(region_locks_map);

    

    // Create in-memory resume state protected by a Mutex and a background flusher
    let resume = Arc::new(Mutex::new(if let Some(path) = &args.resume_file { ResumeState::load(path) } else { ResumeState { processed_tiles: Vec::new() } }));
    let resume_path = args.resume_file.clone();
    let flusher_done = Arc::new(std::sync::atomic::AtomicBool::new(false));
    if let Some(path_clone) = resume_path.clone() {
        let resume_clone = Arc::clone(&resume);
        let done_clone = Arc::clone(&flusher_done);
        std::thread::spawn(move || {
            while !done_clone.load(std::sync::atomic::Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_secs(5));
                let r = resume_clone.lock().unwrap();
                let _ = serde_json::to_string(&*r).map(|s| std::fs::write(&path_clone, s));
            }
            // final flush
            let r = resume_clone.lock().unwrap();
            let _ = serde_json::to_string(&*r).map(|s| std::fs::write(&path_clone, s));
        });
    }

    // Run tiles sequentially to simplify concurrency: this is safe and easier to reason about.
    for tile in tiles.into_iter() {
        // Check resume
        {
            let r = resume.lock().unwrap();
            if r.processed_tiles.contains(&tile.id) {
                tile_pb.inc(1);
                continue;
            }
        }

        let elems = partitions.get(&tile.id).cloned().unwrap_or_default();
        let tile_bbox_xz = tile_bbox(&tile);
        let mut editor = WorldEditor::new(&region_dir, &tile_bbox_xz);
        editor.set_ground(&ground);

        for element in &elems {
            match element {
                ProcessedElement::Way(way) => {
                    if way.tags.contains_key("building") || way.tags.contains_key("building:part") {
                        buildings::generate_buildings(&mut editor, way, args, None);
                    } else if way.tags.contains_key("highway") {
                        highways::generate_highways(&mut editor, element, args);
                    } else if way.tags.contains_key("landuse") {
                        landuse::generate_landuse(&mut editor, way, args);
                    } else if way.tags.contains_key("natural") {
                        natural::generate_natural(&mut editor, element, args);
                    } else if way.tags.contains_key("amenity") {
                        amenities::generate_amenities(&mut editor, element, args);
                    } else if way.tags.contains_key("leisure") {
                        leisure::generate_leisure(&mut editor, way, args);
                    } else if way.tags.contains_key("barrier") {
                        barriers::generate_barriers(&mut editor, element);
                    } else if way.tags.contains_key("waterway") {
                        waterways::generate_waterways(&mut editor, way);
                    } else if way.tags.contains_key("bridge") {
                        //bridges::generate_bridges(&mut editor, way, ground_level); // TODO FIX
                    } else if way.tags.contains_key("railway") {
                        railways::generate_railways(&mut editor, way);
                    } else if way.tags.contains_key("aeroway") || way.tags.contains_key("area:aeroway") {
                        highways::generate_aeroway(&mut editor, way, args);
                    } else if way.tags.get("service") == Some(&"siding".to_string()) {
                        highways::generate_siding(&mut editor, way);
                    } else if way.tags.contains_key("man_made") {
                        man_made::generate_man_made(&mut editor, element, args);
                    }
                }
                ProcessedElement::Node(node) => {
                    if node.tags.contains_key("door") || node.tags.contains_key("entrance") {
                        doors::generate_doors(&mut editor, node);
                    } else if node.tags.contains_key("natural") && node.tags.get("natural") == Some(&"tree".to_string()) {
                        natural::generate_natural(&mut editor, element, args);
                    } else if node.tags.contains_key("amenity") {
                        amenities::generate_amenities(&mut editor, element, args);
                    } else if node.tags.contains_key("barrier") {
                        barriers::generate_barrier_nodes(&mut editor, node);
                    } else if node.tags.contains_key("highway") {
                        highways::generate_highways(&mut editor, element, args);
                    } else if node.tags.contains_key("tourism") {
                        tourisms::generate_tourisms(&mut editor, node);
                    } else if node.tags.contains_key("man_made") {
                        man_made::generate_man_made_nodes(&mut editor, node);
                    }
                }
                ProcessedElement::Relation(rel) => {
                    if rel.tags.contains_key("building") || rel.tags.contains_key("building:part") {
                        buildings::generate_building_from_relation(&mut editor, rel, args);
                    } else if rel.tags.contains_key("water") || rel.tags.get("natural") == Some(&"water".to_string()) {
                        water_areas::generate_water_areas(&mut editor, rel);
                    } else if rel.tags.contains_key("natural") {
                        natural::generate_natural_from_relation(&mut editor, rel, args);
                    } else if rel.tags.contains_key("landuse") {
                        landuse::generate_landuse_from_relation(&mut editor, rel, args);
                    } else if rel.tags.get("leisure") == Some(&"park".to_string()) {
                        leisure::generate_leisure_from_relation(&mut editor, rel, args);
                    } else if rel.tags.contains_key("man_made") {
                        man_made::generate_man_made(&mut editor, &ProcessedElement::Relation(rel.clone()), args);
                    }
                }
            }
        }

        // Generate ground for the tile area
        for x in tile.min_x..=tile.max_x {
            for z in tile.min_z..=tile.max_z {
                if !editor.check_for_block(x, 0, z, Some(&[crate::block_definitions::STONE])) {
                    editor.set_block(crate::block_definitions::GRASS_BLOCK, x, 0, z, None, None);
                    editor.set_block(crate::block_definitions::DIRT, x, -1, z, None, None);
                    editor.set_block(crate::block_definitions::DIRT, x, -2, z, None, None);
                }

                if args.fillground {
                    editor.fill_blocks_absolute(
                        crate::block_definitions::STONE,
                        x,
                        MIN_Y + 1,
                        z,
                        x,
                        editor.get_absolute_y(x, -3, z),
                        z,
                        None,
                        None,
                    );
                }

                editor.set_block_absolute(crate::block_definitions::BEDROCK, x, MIN_Y, z, None, Some(&[crate::block_definitions::BEDROCK]));
            }
        }

        // Acquire region locks for this tile before saving
        let rx_min = tile.min_x >> 9;
        let rx_max = tile.max_x >> 9;
        let rz_min = tile.min_z >> 9;
        let rz_max = tile.max_z >> 9;

        // Collect locks in a deterministic order to avoid deadlocks
        let mut keys: Vec<(i32, i32)> = Vec::new();
        for rx in rx_min..=rx_max {
            for rz in rz_min..=rz_max {
                keys.push((rx, rz));
            }
        }
        keys.sort();

        // Clone Arcs first so they live for the lifetime of the guards
        let lock_arcs: Vec<Arc<Mutex<()>>> = keys
            .iter()
            .filter_map(|k| region_locks.get(k).cloned())
            .collect();

        let mut guards = Vec::new();
        for arc in &lock_arcs {
            guards.push(arc.lock().unwrap());
        }

        // Save region files for this tile while holding the locks
        editor.save();

        // Update resume state in-memory
        if let Some(path) = &args.resume_file {
            let mut r = resume.lock().unwrap();
            r.processed_tiles.push(tile.id);
            // flush immediately small files to disk to survive crashes
            let _ = serde_json::to_string(&*r).map(|s| std::fs::write(path, s));
        }

        tile_pb.inc(1);
    }
        // Signal flusher to finish and give it a moment
        flusher_done.store(true, std::sync::atomic::Ordering::Relaxed);
        std::thread::sleep(std::time::Duration::from_millis(200));

        emit_gui_progress_update(100.0, "Done! World generation completed.");
        println!("{}", "Done! World generation completed.".green().bold());
        Ok(())
    }
    
