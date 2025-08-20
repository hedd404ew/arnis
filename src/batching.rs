use crate::coordinate_system::cartesian::{XZBBox, XZPoint};
use crate::osm_parser::ProcessedElement;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tile {
    pub min_x: i32,
    pub min_z: i32,
    pub max_x: i32,
    pub max_z: i32,
    pub id: (i32, i32),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coordinate_system::cartesian::XZBBox;

    #[test]
    fn test_split_and_partition() {
        let bbox = XZBBox::rect_from_xz_lengths(63.0, 63.0).unwrap();
        let tiles = split_bbox_into_tiles(&bbox, 32);
        assert!(!tiles.is_empty());
        // Expect 2x2 tiles
        assert_eq!(tiles.len(), 4);

        // Create a synthetic element: a node at x=10,z=10
        use crate::osm_parser::ProcessedNode;
        let node = ProcessedNode { id: 1, tags: std::collections::HashMap::new(), x: 10, z: 10 };
        let elem = crate::osm_parser::ProcessedElement::Node(node);

        let parts = partition_elements_by_tile(&[elem.clone()], &tiles);
        // Should be assigned to exactly 1 tile
        let assigned = parts.values().map(|v| v.len()).sum::<usize>();
        assert_eq!(assigned, 1);
    }
}

impl Tile {
    pub fn contains_point(&self, x: i32, z: i32) -> bool {
        x >= self.min_x && x <= self.max_x && z >= self.min_z && z <= self.max_z
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ResumeState {
    pub processed_tiles: Vec<(i32, i32)>,
}

impl ResumeState {
    pub fn load(path: &str) -> Self {
        if let Ok(s) = fs::read_to_string(path) {
            if let Ok(state) = serde_json::from_str(&s) {
                return state;
            }
        }
        ResumeState {
            processed_tiles: Vec::new(),
        }
    }

    pub fn save(&self, path: &str) {
        if let Ok(s) = serde_json::to_string(self) {
            let _ = fs::write(path, s);
        }
    }
}

/// Split a bounding box into square tiles of given `tile_size` (in blocks).
pub fn split_bbox_into_tiles(xzbbox: &XZBBox, tile_size: i32) -> Vec<Tile> {
    let mut tiles = Vec::new();

    let min_x = xzbbox.min_x();
    let max_x = xzbbox.max_x();
    let min_z = xzbbox.min_z();
    let max_z = xzbbox.max_z();

    let nx = ((max_x - min_x) / tile_size).max(0) + 1;
    let nz = ((max_z - min_z) / tile_size).max(0) + 1;

    for ix in 0..nx {
        for iz in 0..nz {
            let tx_min_x = min_x + ix * tile_size;
            let tx_min_z = min_z + iz * tile_size;
            let tx_max_x = (tx_min_x + tile_size - 1).min(max_x);
            let tx_max_z = (tx_min_z + tile_size - 1).min(max_z);

            tiles.push(Tile {
                min_x: tx_min_x,
                min_z: tx_min_z,
                max_x: tx_max_x,
                max_z: tx_max_z,
                id: (ix, iz),
            });
        }
    }

    tiles
}

/// Partition processed elements into tile buckets. Elements are assigned to any tile that
/// contains at least one of their nodes. Relations are assigned if any member way's nodes
/// are inside the tile.
pub fn partition_elements_by_tile(
    elements: &[ProcessedElement],
    tiles: &[Tile],
) -> HashMap<(i32, i32), Vec<ProcessedElement>> {
    let mut map: HashMap<(i32, i32), Vec<ProcessedElement>> = HashMap::new();

    for e in elements.iter() {
        // gather candidate tiles
        let mut assigned = false;
        for tile in tiles.iter() {
            let mut has_point = false;
            for node in e.nodes() {
                let p: XZPoint = node.xz();
                if tile.contains_point(p.x, p.z) {
                    has_point = true;
                    break;
                }
            }

            if has_point {
                map.entry(tile.id).or_default().push(e.clone());
                assigned = true;
            }
        }

        // Fallback: if element had no nodes (rare), assign to first tile
        if !assigned {
            if let Some(first) = tiles.first() {
                map.entry(first.id).or_default().push(e.clone());
            }
        }
    }

    map
}

/// Helper to get tile bbox from a tile id
pub fn tile_bbox(tile: &Tile) -> XZBBox {
    // Build a rectangle bbox by creating a rect at origin with correct lengths
    // and translating it to the tile's min coordinates. This avoids referencing
    // internal modules.
    let length_x = (tile.max_x - tile.min_x) as f64;
    let length_z = (tile.max_z - tile.min_z) as f64;

    let mut bbox = XZBBox::rect_from_xz_lengths(length_x, length_z)
        .expect("Invalid tile lengths");

    // Translate bbox to tile.min coordinates
    let translate = crate::coordinate_system::cartesian::XZVector { dx: tile.min_x, dz: tile.min_z };
    bbox += translate;

    bbox
}
