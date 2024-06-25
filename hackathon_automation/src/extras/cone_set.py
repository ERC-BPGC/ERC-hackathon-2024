import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

def generate_construction_cone_xml(x, y, id_number, color='red'):
    # Create the root element
    model = ET.Element("model", name=f"Construction_Cone_id_{id_number}")
    
    # Create the link element
    link = ET.SubElement(model, "link", name="link")
    
    # Add child elements to link
    ET.SubElement(link, "self_collide").text = "0"
    
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "pose").text = "0 0 0 0 -0 0"
    inertia = ET.SubElement(inertial, "inertia")
    ET.SubElement(inertia, "ixx").text = "1"
    ET.SubElement(inertia, "ixy").text = "0"
    ET.SubElement(inertia, "ixz").text = "0"
    ET.SubElement(inertia, "iyy").text = "1"
    ET.SubElement(inertia, "iyz").text = "0"
    ET.SubElement(inertia, "izz").text = "1"
    ET.SubElement(inertial, "mass").text = "1"
    
    ET.SubElement(link, "enable_wind").text = "0"
    ET.SubElement(link, "kinematic").text = "0"
    ET.SubElement(link, "pose").text = "0 0 0 0 -0 0"
    ET.SubElement(link, "gravity").text = "1"
    
    # Visual element
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    ET.SubElement(mesh, "scale").text = "3.27166 3.27166 3.27166"
    ET.SubElement(mesh, "uri").text = "model://construction_cone/meshes/construction_cone.dae"
    
    material = ET.SubElement(visual, "material")
    shader = ET.SubElement(material, "shader", type="pixel")
    ET.SubElement(shader, "normal_map").text = "__default__"
    
    # Set color based on input
    if color.lower() == 'blue':
        color_value = "0 0 0.643 1"
    else:  # default to red
        color_value = "0.643 0 0 1"
    
    ET.SubElement(material, "ambient").text = color_value
    ET.SubElement(material, "diffuse").text = color_value
    ET.SubElement(material, "specular").text = color_value
    ET.SubElement(material, "emissive").text = "0 0 0 1"
    
    ET.SubElement(visual, "pose").text = "0 0 0 0 -0 0"
    ET.SubElement(visual, "transparency").text = "0"
    ET.SubElement(visual, "cast_shadows").text = "1"
    
    # Collision element
    collision = ET.SubElement(link, "collision", name="collision")
    ET.SubElement(collision, "laser_retro").text = "0"
    ET.SubElement(collision, "max_contacts").text = "10"
    ET.SubElement(collision, "pose").text = "0 0 0 0 -0 0"
    
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_mesh = ET.SubElement(collision_geometry, "mesh")
    ET.SubElement(collision_mesh, "uri").text = "model://construction_cone/meshes/construction_cone.dae"
    ET.SubElement(collision_mesh, "scale").text = "3.27166 3.27166 3.27166"
    
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    ET.SubElement(ode, "mu").text = "1"
    ET.SubElement(ode, "mu2").text = "1"
    ET.SubElement(ode, "fdir1").text = "0 0 0"
    ET.SubElement(ode, "slip1").text = "0"
    ET.SubElement(ode, "slip2").text = "0"
    
    torsional = ET.SubElement(friction, "torsional")
    ET.SubElement(torsional, "coefficient").text = "1"
    ET.SubElement(torsional, "patch_radius").text = "0"
    ET.SubElement(torsional, "surface_radius").text = "0"
    ET.SubElement(torsional, "use_patch_radius").text = "1"
    torsional_ode = ET.SubElement(torsional, "ode")
    ET.SubElement(torsional_ode, "slip").text = "0"
    
    bounce = ET.SubElement(surface, "bounce")
    ET.SubElement(bounce, "restitution_coefficient").text = "0"
    ET.SubElement(bounce, "threshold").text = "1e+06"
    
    contact = ET.SubElement(surface, "contact")
    ET.SubElement(contact, "collide_without_contact").text = "0"
    ET.SubElement(contact, "collide_without_contact_bitmask").text = "1"
    ET.SubElement(contact, "collide_bitmask").text = "1"
    contact_ode = ET.SubElement(contact, "ode")
    ET.SubElement(contact_ode, "soft_cfm").text = "0"
    ET.SubElement(contact_ode, "soft_erp").text = "0.2"
    ET.SubElement(contact_ode, "kp").text = "1e+13"
    ET.SubElement(contact_ode, "kd").text = "1"
    ET.SubElement(contact_ode, "max_vel").text = "0.01"
    ET.SubElement(contact_ode, "min_depth").text = "0"
    
    bullet = ET.SubElement(contact, "bullet")
    ET.SubElement(bullet, "split_impulse").text = "1"
    ET.SubElement(bullet, "split_impulse_penetration_threshold").text = "-0.01"
    ET.SubElement(bullet, "soft_cfm").text = "0"
    ET.SubElement(bullet, "soft_erp").text = "0.2"
    ET.SubElement(bullet, "kp").text = "1e+13"
    ET.SubElement(bullet, "kd").text = "1"
    
    ET.SubElement(model, "static").text = "0"
    ET.SubElement(model, "allow_auto_disable").text = "1"
    ET.SubElement(model, "pose").text = f"{x} {y} 0 0 -0 0"
    
    # Convert the XML to a string with proper formatting
    xml_string = ET.tostring(model, encoding="unicode")
    dom = minidom.parseString(xml_string)
    pretty_xml = dom.toprettyxml(indent="  ")
    
    return pretty_xml

# Example usage
x_coord = 10.395211
y_coord = 7.784442
id_num = 5

# Generate a red cone (default)
xml_output_red = generate_construction_cone_xml(x_coord, y_coord, id_num)
# print("Red Cone XML:")
print(xml_output_red)

# Generate a blue cone
xml_output_blue = generate_construction_cone_xml(x_coord, y_coord, id_num + 1, color='blue')
# print("\nBlue Cone XML:")
# print(xml_output_blue)