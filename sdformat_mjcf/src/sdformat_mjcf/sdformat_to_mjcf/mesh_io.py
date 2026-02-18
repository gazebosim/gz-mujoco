from dataclasses import dataclass
import numpy as np
import os
import sys
import trimesh


@dataclass
class Material:
    specular: float
    shininess: float
    rgba: list[float]  # each channel has a value from 0 to 1
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
    default_rgba = [0.5, 0.5, 0.5, 1.0]  # Default grey

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
        rgba = [float(c) for c in color]
    elif isinstance(color, (list, tuple)) and len(color) == 3:
        # RGB, add alpha channel
        rgba = [float(c) for c in color] + [1.0]
    else:
        # Final fallback for unexpected format
        rgba = default_rgba

    return Material(
        specular=specular,
        shininess=shininess,
        rgba=rgba
    ), image


def convert_mesh_to_obj_multimesh(input_filepath: str,
                                  output_filepath: str) -> ConversionOutput:
    """
    Converts an input mesh file (e.g., glb, glTF) into multiple OBJ files,
    one for each mesh in the input scene.

    Args:
        input_filepath (str): Path to the input mesh file.
        output_filepath (str): Base path for output files. Each mesh will be
                               saved as <base_path>_<mesh_name>.obj

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

        # Apply the transform to the mesh's vertices *before* exporting the
        # OBJ. This writes the mesh at its correctly scaled/positioned size
        # to the OBJ.
        transformed_mesh = mesh.copy()
        transformed_mesh.apply_transform(transform_matrix)

        # Use the geometry name and path for a unique filename
        if len(model.geometry) > 1:
            safe_name = (
                G_name.replace(' ', '_').replace('/', '_').replace('\\', '_')
            )
            obj_filename = f"{base_path}_{safe_name}_{count}.obj"
            count += 1
        else:
            obj_filename = output_filepath

        try:
            with open(obj_filename, 'wb') as f:
                # Export the transformed mesh
                transformed_mesh.export(f, file_type='obj')

            # 3. EXTRACT MATERIAL AND COMPILE RESULT
            extracted_material = None
            extracted_image = None
            if hasattr(mesh.visual, "material") and mesh.visual.material is not None:
                # Use the material from the original mesh
                extracted_material, extracted_image = _extract_material_info(
                    mesh.visual.material)

            if extracted_material is None:
                extracted_material = Material(specular=0.5, shininess=30.0,
                                              rgba=[0.5, 0.5, 0.5, 1.0])

            if extracted_image is not None:
                try:
                    texture_filename = os.path.splitext(obj_filename)[0] + ".png"
                    print(f"Exporting texture to {texture_filename}")
                    extracted_image.save(texture_filename)
                    extracted_material.texture = texture_filename
                except Exception as e:
                    print(f"WARNING: Failed to export texture for mesh '{G_name}'.")
                    print(f"Detail: {e}")

            mesh_info = MeshInfo(mat=extracted_material)

            # Store result
            conversion_results[obj_filename] = mesh_info

        except Exception as e:
            print(f"\nERROR: Failed to export mesh '{G_name}' to OBJ.")
            print(f"Detail: {e}")
            continue

    return ConversionOutput(obj_files=conversion_results)
