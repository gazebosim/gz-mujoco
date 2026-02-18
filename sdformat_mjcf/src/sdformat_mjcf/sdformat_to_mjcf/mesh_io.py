from dataclasses import dataclass
import numpy as np
import os
import sys
import trimesh
import hashlib


@dataclass(frozen=True)
class Material:
    specular: float
    shininess: float
    rgba: tuple[float]  # each channel has a value from 0 to 1
    texture: str = None


@dataclass
class MeshInfo:
    mat: Material


@dataclass
class ConversionOutput:
    # Dictionary with key = obj filename for mesh, value = assigned material
    obj_files: dict[str, MeshInfo]


def _extract_material_info(mesh_material) -> tuple[Material, object]:
    """
    Helper function to extract key material properties from a trimesh material
    and map them to the desired Material dataclass structure.

    NOTE: Removed explicit type hint for mesh_material to avoid the
    'BaseMaterial' error, relying on runtime checks instead.
    """
    default_rgba = (0.5, 0.5, 0.5, 1.0)  # Default grey

    # Simple defaults for common material properties
    # TODO: Make this configurable
    specular = 0.5
    shininess = 30.0
    color = default_rgba
    image = None

    # Try to extract color information by checking against known material types
    if isinstance(mesh_material, trimesh.visual.material.PBRMaterial):
        # PBRMaterial uses baseColorFactor
        color = mesh_material.baseColorFactor
        if mesh_material.baseColorTexture is not None:
            image = mesh_material.baseColorTexture
    elif isinstance(mesh_material, trimesh.visual.material.SimpleMaterial):
        # SimpleMaterial usually has a diffuse color
        color = mesh_material.diffuse
        if mesh_material.image is not None:
            image = mesh_material.image
    elif hasattr(mesh_material, 'main_color'):
        # Fallback for older/custom materials
        color = mesh_material.main_color
        if hasattr(mesh_material, 'image') and mesh_material.image is not None:
            image = mesh_material.image

    # --- Color Standardization ---
    if isinstance(color, np.ndarray):
        # trimesh specifies colors in uint8.
        color = color.astype(np.float32)
        color /= 255.0
        color = color.tolist()

    if isinstance(color, (list, tuple)) and len(color) == 4:
        # RGBA already
        rgba = tuple(float(c) for c in color)
    elif isinstance(color, (list, tuple)) and len(color) == 3:
        # RGB, add alpha channel
        rgba = tuple([float(c) for c in color] + [1.0])
    else:
        # Final fallback for unexpected format
        rgba = default_rgba

    return Material(
        specular=specular,
        shininess=shininess,
        rgba=rgba
    ), image


def convert_mesh_to_obj_multimesh(input_filepath: str,
                                  output_filepath: str,
                                  merge_same_material: bool = False) -> ConversionOutput:
    """
    Converts an input mesh file (e.g., glb, glTF) into multiple OBJ files.
    
    If merge_same_material is True, meshes sharing the same visual material
    will be concatenated into a single OBJ file, reducing the number of files
    and MuJoCo geoms. This is recommended for visual geometries.

    Args:
        input_filepath (str): Path to the input mesh file.
        output_filepath (str): Base path for output files. Each mesh will be
                               saved as <base_path>_<mesh_name>.obj
        merge_same_material (bool): Whether to merge meshes with identical materials.

    Returns:
        ConversionOutput: An object containing a dictionary mapping the
                          generated OBJ filenames to their extracted material
                          properties.
    """
    if not os.path.exists(input_filepath):
        raise RuntimeError(
            f"Unable to find the input mesh file {input_filepath}"
        )

    try:
        model = trimesh.load(input_filepath, force='scene')
    except Exception as e:
        print(f"\nERROR: Failed to load file: {input_filepath}. Check if the "
              "file is valid.")
        print(f"Detail: {e}")
        sys.exit(1)

    if not isinstance(model, trimesh.Scene) or not model.geometry:
        print(f"\nERROR: Loaded model object type is unexpected: "
              f"{type(model)}. Conversion failed.")
        sys.exit(1)

    base_path, _ = os.path.splitext(output_filepath)
    conversion_results: dict[str, MeshInfo] = {}
    saved_textures: dict[str, str] = {}  # hash -> filename
    saved_texture_ids: dict[int, str] = {}  # id(image) -> filename

    # Groups for merging: key -> list of transformed meshes
    # key structure: (Material, image_id) or (Material, image_id, unique_counter)
    mesh_groups = {}
    
    # Store auxiliary data for each group to avoid re-extraction
    group_data = {} # key -> {'material': Material, 'image': Image, 'names': list}

    # Iterate over the scene's NODES to capture transforms
    count = 0
    for _, TG_tuple in model.graph.to_flattened().items():
        if not TG_tuple["geometry"]:
            continue
        G_name = TG_tuple["geometry"]
        transform_matrix = TG_tuple["transform"]

        # Check if the geometry name is in the loaded geometry map
        if G_name not in model.geometry:
            print(f"WARNING: Geometry '{G_name}' referenced in graph but not "
                  "in geometry map. Skipping.")
            continue

        mesh = model.geometry[G_name]

        if not isinstance(mesh, trimesh.Trimesh):
            print(f"Skipping geometry '{G_name}' as it's not a Trimesh "
                  "object.")
            continue

        # Apply the transform to the mesh's vertices
        transformed_mesh = mesh.copy()
        transformed_mesh.apply_transform(transform_matrix)
        
        # Extract material
        extracted_material = None
        extracted_image = None
        if hasattr(mesh.visual, "material") and mesh.visual.material is not None:
             extracted_material, extracted_image = _extract_material_info(mesh.visual.material)
        
        if extracted_material is None:
             extracted_material = Material(specular=0.5, shininess=30.0,
                                           rgba=(0.5, 0.5, 0.5, 1.0))

        # Determine grouping key
        image_id = id(extracted_image) if extracted_image is not None else None
        
        if merge_same_material:
            # Group by material properties and image identity
            key = (extracted_material, image_id)
        else:
            # Unique group for every mesh
            key = (extracted_material, image_id, count)
            count += 1
            
        if key not in mesh_groups:
            mesh_groups[key] = []
            group_data[key] = {
                'material': extracted_material,
                'image': extracted_image,
                'names': []
            }
            
        mesh_groups[key].append(transformed_mesh)
        group_data[key]['names'].append(G_name)

    # Process each group
    group_counter = 0
    for key, meshes in mesh_groups.items():
        data = group_data[key]
        extracted_material = data['material']
        extracted_image = data['image']
        names = data['names']
        
        # Merge meshes if multiple
        if len(meshes) > 1:
            # Use trimesh to concatenate
            final_mesh = trimesh.util.concatenate(meshes)
            # Use a descriptive name if possible, or generic
            # If merging 32 meshes, naming is tricky. Use first one or hash.
            # Using count to ensure uniqueness of file
            suffix = f"merged_{group_counter}"
        else:
            final_mesh = meshes[0]
            # Clean name from original mesh
            suffix = names[0].replace(' ', '_').replace('/', '_').replace('\\', '_')
            if not merge_same_material:
                 # Restore original suffix behavior if not merging (handled by key having count)
                 # Actually key has count, so loop order is stable.
                 # But we need to match original naming scheme roughly if possible, but simpler is better.
                 pass

        # If not merging, we might have name collisions if we don't use unique ID.
        # But 'names[0]' is unique per node? No, 'G_name' is geometry name.
        # If geometry is instanced multiple times, G_name is same.
        # So we must use a unique suffix.
        if merge_same_material:
             obj_filename = f"{base_path}_{suffix}.obj"
        else:
             # Using the unique index from the key or counter
             obj_filename = f"{base_path}_{suffix}_{group_counter}.obj"
        
        group_counter += 1
        
        try:
            with open(obj_filename, 'wb') as f:
                print("Exporting", obj_filename)
                final_mesh.export(f, file_type='obj')
            print("Exported", f)

            # Handle texture
            if extracted_image is not None:
                try:
                    # Optimization: Check if we've already processed this exact image object
                    image_id = id(extracted_image)
                    if image_id in saved_texture_ids:
                        texture_filename = saved_texture_ids[image_id]
                        print(f"Reusing texture object {texture_filename}")
                    else:
                        # Fallback: Check if content is identical
                        image_bytes = extracted_image.tobytes()
                        image_hash = hashlib.sha256(image_bytes).hexdigest()[:8]
                        
                        if image_hash in saved_textures:
                            texture_filename = saved_textures[image_hash]
                            print(f"Reusing texture content {texture_filename}")
                        else:
                            texture_filename = f"{base_path}_{image_hash}.png"
                            print(f"Exporting texture to {texture_filename}")
                            extracted_image.save(texture_filename)
                            saved_textures[image_hash] = texture_filename
                        
                        saved_texture_ids[image_id] = texture_filename
                    
                    # Update material with texture filename
                    # Note: We need a new Material object because it's frozen
                    extracted_material = Material(
                        specular=extracted_material.specular,
                        shininess=extracted_material.shininess,
                        rgba=extracted_material.rgba,
                        texture=texture_filename
                    )
                except Exception as e:
                    print(f"WARNING: Failed to process texture.")
                    print(f"Detail: {e}")

            mesh_info = MeshInfo(mat=extracted_material)
            conversion_results[obj_filename] = mesh_info

        except Exception as e:
            print(f"\nERROR: Failed to export mesh group to OBJ.")
            print(f"Detail: {e}")
            continue

    return ConversionOutput(obj_files=conversion_results)
