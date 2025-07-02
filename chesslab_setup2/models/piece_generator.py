import bpy
import os
import glob
import math 
import time 

piece_to_size = { # in m, [width, height, depth]
"pawn":[0.03,0.03,0.04],
"bishop":[0.03,0.03,0.062],
"king":[0.03,0.03,0.08],
"queen":[0.03,0.03,0.08],
"knight":[0.03,0.03,0.062],
"rook":[0.03,0.03,0.062]
}

# LEO INSTRUCTIONS: 
# 1. canviar el front_face a 3. (un cop tinguis tots els front faces .png al directory "meshes" de cada peça. (mateix que l'aruco))
# 2. canviar el path_to_models, de manera que apunti on tu tens el cheslab_setup/models.
# 3. Guarda aquests fitxer de "text" amb "Alt+S" i quan estiguis llest per executar prem "Alt+P"
# 4. Se t'ha creat la carpeta "test" al teu path_to_models amb els resultats! Comprova que tot esta be, si es aixi treu el "test" del path de la ultima linea on s'exporta el collada.
# 5. Repeteix el pas 3. 

top_face = 5 # face index to add aruco should be 4!
front_face = 1 # face index to add front image sould be 3!
black_color = (0.25, 0.16, 0.08, 1)
white_color = (0.96, 0.91, 0.86, 1) 

path_to_models = "/home/users/leopold.palomo/docencia/Introduction_to_ROS/student/git-repos/chesslab_setup/models"
def load_image_once(image_path):
    image_name = os.path.basename(image_path)
    for image in bpy.data.images: 
        if image.name.startswith(image_name):
            return image 
    return bpy.data.images.load(image_path)

def create_image_material(image, image_name):
    mat = bpy.data.materials.new(name=image_name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    tex_image = mat.node_tree.nodes.new("ShaderNodeTexImage")
    tex_image.image = image
    mat.node_tree.links.new(bsdf.inputs["Base Color"], tex_image.outputs["Color"])
    return mat

def create_color_material(color): #"white": (0.7, 0.4, 0.3, 1) / "black": (0.3, 0.2, 0.1, 1)
    mat = bpy.data.materials.new(name="color_material")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs['Base Color'].default_value = color
    return mat 
  

def assign_materials_to_faces(obj, model_dir):
    # Get paths
    textures_path = os.path.join(path_to_models, model_dir, "meshes")
    # aruco:
    aruco_path = glob.glob(os.path.join(textures_path, "aruco*.png"))
    # front:
    front_path = glob.glob(os.path.join(textures_path,"front*.png"))
    # base_color:
    if "B" in model_dir:  # black piece
        base_color = black_color
        
    elif "W" in model_dir:  # white_piece
        base_color = white_color 
        
    # Unwrap the UV of the object
    #bpy.ops.object.mode_set(mode='EDIT')
    #bpy.ops.mesh.select_all(action='SELECT')
    #bpy.ops.uv.unwrap(method='ANGLE_BASED', margin=0.001)
    #bpy.ops.object.mode_set(mode='OBJECT')

    # Assign textures to faces
    for i, face in enumerate(obj.data.polygons):
        if i == top_face:  # TOP
            texture = load_image_once(aruco_path[0])
            mat = create_image_material(texture, os.path.basename(aruco_path[0]))
            
        elif i == front_face: # FRONT
            texture = load_image_once(front_path[0])
            mat = create_image_material(texture, os.path.basename(front_path[0]))
                
        else:  # Other faces  a simple color 
            mat = create_color_material(base_color)
            
        obj.data.materials.append(mat)
        face.material_index = len(obj.material_slots) - 1
        obj.data.update()


def adjust_uv_map(face_index):
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.mesh.select_mode(type='FACE')

    # Access the mesh data and select the face
    bpy.ops.object.mode_set(mode='OBJECT')
    mesh = bpy.context.active_object.data
    mesh.polygons[face_index].select = True
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.uv.unwrap(method='ANGLE_BASED', margin=0.000000000000000001)

def rotate_face(face_index):
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.mesh.select_mode(type='FACE')

    # Access the mesh data and select the face
    bpy.ops.object.mode_set(mode='OBJECT')
    mesh = bpy.context.active_object.data
    mesh.polygons[face_index].select = True
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.object.vertex_group_select()
    #bpy.ops.transform.rotate(value=math.pi/2.0, orient_axis='Z', orient_type='GLOBAL')


def main():
    models = os.listdir(path_to_models)
    for model_dir in models:
        for piece in piece_to_size.keys():
            if piece in model_dir:
                
                #bpy.ops.scene.delete()
                bpy.ops.scene.new(type='EMPTY')

                # Add a cube and rescale
                bpy.ops.mesh.primitive_cube_add(size=1)
                cube = bpy.context.active_object
                cube.scale = tuple(piece_to_size[piece])  # size it
                bpy.ops.object.transform_apply(location=True, rotation=True, scale=True, properties=True, isolate_users=False)  # Apply scale
                
                # Asign face colors
                assign_materials_to_faces(cube, model_dir)
                # Adjust uv for the texture faces 
                adjust_uv_map(top_face) # adjust top face uv  
                adjust_uv_map(front_face) # adjust front face uv 
                rotate_face(front_face)
                if piece in ["rook", "bishop", "knight"]:
                    bpy.ops.object.mode_set(mode='OBJECT')
                    cube = bpy.context.active_object
                    cube.scale = tuple((1.0,1.0, 0.06/piece_to_size[piece][2]))   # scale it
                    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True, properties=True, isolate_users=False)  # Apply scale
                # correct rotation to match desired indiexes
                #bpy.context.object.rotation_euler[0] = 3.1415
                bpy.context.object.rotation_euler[2] = 3.1415
                
                # Export the mesh:
                bpy.ops.wm.collada_export(filepath=os.path.join(path_to_models,"test", model_dir, "meshes",f"{model_dir}.dae"))
if __name__=="__main__":
    main()
