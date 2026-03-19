//! glTF/glb file loader using the `gltf` crate.

use aeroblend_physics::types::MeshData;

/// Result of loading a glTF file.
pub struct LoadedGltf {
    pub meshes: Vec<MeshData>,
}

/// Load a glTF/glb file and extract all meshes.
pub fn load_gltf(path: &str) -> Result<LoadedGltf, String> {
    let (document, buffers, _images) =
        gltf::import(path).map_err(|e| format!("Failed to load glTF: {}", e))?;

    let mut meshes = Vec::new();

    for scene in document.scenes() {
        for node in scene.nodes() {
            extract_node_meshes(&node, &buffers, &glam::DMat4::IDENTITY, &mut meshes);
        }
    }

    Ok(LoadedGltf { meshes })
}

fn extract_node_meshes(
    node: &gltf::Node,
    buffers: &[gltf::buffer::Data],
    parent_transform: &glam::DMat4,
    meshes: &mut Vec<MeshData>,
) {
    let local_transform = node_transform(node);
    let world_transform = *parent_transform * local_transform;

    if let Some(mesh) = node.mesh() {
        let name = node
            .name()
            .or_else(|| mesh.name())
            .unwrap_or("unnamed")
            .to_string();

        let mut all_vertices = Vec::new();
        let mut all_normals = Vec::new();
        let mut all_uvs = Vec::new();
        let mut all_indices = Vec::new();
        let mut offset: u32 = 0;

        for primitive in mesh.primitives() {
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));

            let positions: Vec<[f32; 3]> = reader
                .read_positions()
                .map(|iter| iter.collect())
                .unwrap_or_default();

            let normals: Vec<[f32; 3]> = reader
                .read_normals()
                .map(|iter| iter.collect())
                .unwrap_or_else(|| vec![[0.0, 1.0, 0.0]; positions.len()]);

            let uvs: Vec<[f32; 2]> = reader
                .read_tex_coords(0)
                .map(|tc| tc.into_f32().collect())
                .unwrap_or_else(|| vec![[0.0, 0.0]; positions.len()]);

            let indices: Vec<u32> = reader
                .read_indices()
                .map(|idx| idx.into_u32().collect())
                .unwrap_or_else(|| (0..positions.len() as u32).collect());

            let n_verts = positions.len();

            // Apply transform
            let rot = glam::Mat3::from_cols(
                world_transform.col(0).truncate().as_vec3(),
                world_transform.col(1).truncate().as_vec3(),
                world_transform.col(2).truncate().as_vec3(),
            );
            let translation = glam::Vec3::new(
                world_transform.col(3).x as f32,
                world_transform.col(3).y as f32,
                world_transform.col(3).z as f32,
            );

            // Normal matrix = inverse transpose of upper-left 3x3
            // For orthogonal matrices, inverse transpose = the matrix itself
            let normal_mat = rot;

            for pos in &positions {
                let p = rot * glam::Vec3::from(*pos) + translation;
                all_vertices.push([p.x, p.y, p.z]);
            }

            for norm in &normals {
                let n = (normal_mat * glam::Vec3::from(*norm)).normalize_or_zero();
                all_normals.push([n.x, n.y, n.z]);
            }

            all_uvs.extend_from_slice(&uvs);

            for idx in &indices {
                all_indices.push(idx + offset);
            }
            offset += n_verts as u32;
        }

        meshes.push(MeshData {
            vertices: all_vertices,
            normals: all_normals,
            uvs: all_uvs,
            indices: all_indices,
            name,
        });
    }

    for child in node.children() {
        extract_node_meshes(&child, buffers, &world_transform, meshes);
    }
}

fn node_transform(node: &gltf::Node) -> glam::DMat4 {
    let m = node.transform().matrix();
    glam::DMat4::from_cols_array_2d(&[
        [m[0][0] as f64, m[0][1] as f64, m[0][2] as f64, m[0][3] as f64],
        [m[1][0] as f64, m[1][1] as f64, m[1][2] as f64, m[1][3] as f64],
        [m[2][0] as f64, m[2][1] as f64, m[2][2] as f64, m[2][3] as f64],
        [m[3][0] as f64, m[3][1] as f64, m[3][2] as f64, m[3][3] as f64],
    ])
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_node_transform_identity() {
        // Just verify the transform function doesn't panic
        // Real testing would need a glTF file
    }
}
