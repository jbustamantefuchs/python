import cv2, os, numpy as np, time, imutils, re, shutil, subprocess
from imutils import paths
from tqdm import tqdm 
from pathlib import Path

name_obj = ""

def capture_background(): 
    cap = cv2.VideoCapture(0)
    if not cap.isOpened(): return
    while True:
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.resize(frame, (640,480))
        cv2.imshow("Press 's' to capture your background", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            path = f"{os.path.dirname(os.path.abspath(__file__))}/backgrounds"
            os.makedirs(path, exist_ok=True)
            cv2.imwrite(f"{path}/1.png", frame)
            break
        elif key == ord('q'): break
    cap.release(), cv2.destroyAllWindows()
    time.sleep(1)
    print('background')

def nothing(x):
    pass

def capture_object_cam_trackbars(): 
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Ajustar ancho
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Ajustar alto

    cv2.namedWindow('Trackbars')

    for c, v in [('redLow', 0), ('redHigh', 255), 
                 ('greenLow', 0), ('greenHigh', 255), 
                 ('blueLow', 0), ('blueHigh', 255)]:
        cv2.createTrackbar(c, 'Trackbars', v, 255, nothing)

    os.makedirs(os.path.join(os.path.dirname(__file__), "isolated"), exist_ok=True)

    cv2.waitKey(100)  # Permitir que las barras se actualicen

    while True:
        ret, frame = cap.read()
        if not ret: break

        low = np.array([cv2.getTrackbarPos(c, 'Trackbars') for c in ['blueLow', 'greenLow', 'redLow']], dtype=np.uint8)
        high = np.array([cv2.getTrackbarPos(c, 'Trackbars') for c in ['blueHigh', 'greenHigh', 'redHigh']], dtype=np.uint8)

        mask = cv2.inRange(frame, low, high)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("Press 's' to capture", result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('s'):
            path = os.path.join(os.path.dirname(__file__), 'isolated')
            filename = f"{len([f for f in os.listdir(path) if f.endswith('.png')]) + 1}.png"
            cv2.imwrite(os.path.join(path, filename), result)

    cap.release()
    cv2.destroyAllWindows()
    print('Capturing objects')

def rotate_images():
    folder = f"{os.path.dirname(os.path.abspath(__file__))}/isolated"
    for path in paths.list_images(folder):
        img = cv2.imread(path)
        base_name = os.path.basename(path).split('.')[0]
        cv2.imwrite(f"{folder}/{base_name}.png", img)
        for i in range(1, 10):
            angle = -i * 35
            cv2.imwrite(f"{folder}/{base_name}_{i}.png", imutils.rotate_bound(img, angle))
    time.sleep(1)
    print('Rotating isolated pictures')

def combine_images():  

    background_folder = f"{os.path.dirname(os.path.abspath(__file__))}/backgrounds"
    isolated_folder = f"{os.path.dirname(os.path.abspath(__file__))}/isolated"
    output_folder = f"{os.path.dirname(os.path.abspath(__file__))}/combined"
    os.makedirs(output_folder, exist_ok=True)
    print('combining images')
    
    background_images = [f for f in os.listdir(background_folder) if f.endswith(('.jpg', '.png', '.jpeg'))]
    isolated_images = [f for f in os.listdir(isolated_folder) if f.endswith(('.jpg', '.png', '.jpeg'))]
    
    for iso in isolated_images:  # Iterar sobre las imágenes aisladas
        isolated = cv2.imread(os.path.join(isolated_folder, iso), cv2.IMREAD_UNCHANGED)
        if isolated.shape[2] == 3:
            isolated = cv2.cvtColor(isolated, cv2.COLOR_BGR2BGRA)
        isolated[cv2.inRange(isolated, (0, 0, 0, 255), (0, 0, 0, 255)) > 0] = [0, 0, 0, 0]
        isolated_resized = cv2.resize(isolated, (640, 480))
        
        for bg_idx, bg in enumerate(background_images, start=1):  # Iterar sobre los fondos
            background = cv2.resize(cv2.imread(os.path.join(background_folder, bg)), (640, 480))
            
            combined = background.copy()
            for y in range(isolated_resized.shape[0]):
                for x in range(isolated_resized.shape[1]):
                    if isolated_resized[y, x][3] > 0:
                        combined[y, x] = isolated_resized[y, x][:3]
            
            # Generar el nombre del archivo de salida con el nombre de la imagen aislada y una letra
            output_name = f"{os.path.splitext(iso)[0]}_{chr(96 + bg_idx)}.png"
            cv2.imwrite(os.path.join(output_folder, output_name), combined)
    time.sleep(1)

def draw_rectangle():
    isolated_folder = os.path.join(os.path.dirname(__file__), "isolated")
    xml_images_folder = os.path.join(os.path.dirname(__file__), "xml_images")
    os.makedirs(xml_images_folder, exist_ok=True)
    os.makedirs(isolated_folder, exist_ok=True)

    def handle_mouse(event, x, y, flags, param):
        nonlocal ix, iy, drawing, rect_coords
        if event == cv2.EVENT_LBUTTONDOWN:
            img[:] = cv2.imread(img_path)
            drawing, ix, iy = True, x, y
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            cv2.imshow("Draw square to identify the object", cv2.rectangle(img.copy(), (ix, iy), (x, y), (0, 255, 0), 2))
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            rect_coords[:] = [ix, iy, x, y]
            cv2.rectangle(img, (ix, iy), (x, y), (0, 255, 0), 2)
            cv2.imshow("Draw square to identify the object", img)

    for img_file in filter(lambda f: f.endswith('.png') and f.split('.')[0].isdigit(), os.listdir(isolated_folder)):
        img_path = os.path.join(isolated_folder, img_file)
        img = cv2.imread(img_path)
        rect_coords, ix, iy, drawing = [], -1, -1, False 
        cv2.imshow("Draw square to identify the object", img)
        cv2.setMouseCallback("Draw square to identify the object", handle_mouse)
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):  # Si se presiona 'S', se guarda la imagen
                if rect_coords:
                    cv2.imwrite(os.path.join(xml_images_folder, img_file), img)
                break
            elif key == ord('q'):  # Si se presiona 'Q', se sale sin guardar
                break

        cv2.destroyAllWindows()

    time.sleep(1)
    print('Drawing contour for XMLs')


def rotate_xml_images():
    folder = f"{os.path.dirname(os.path.abspath(__file__))}/xml_images"
    for path in paths.list_images(folder):
        img = cv2.imread(path)
        base_name = os.path.basename(path).split('.')[0]
        cv2.imwrite(f"{folder}/{base_name}.png", img)
        for i in range(1, 10):
            angle = -i * 35
            cv2.imwrite(f"{folder}/{base_name}_{i}.png", imutils.rotate_bound(img, angle))
    time.sleep(1)
    print('rotating xml pictures')

def get_coordinates():
    os.makedirs(f"{os.path.dirname(os.path.abspath(__file__))}/data", exist_ok=True)
    with open(f"{os.path.dirname(os.path.abspath(__file__))}/data/coordinates.txt", "w") as f:
        for image_name in os.listdir(f"{os.path.dirname(os.path.abspath(__file__))}/xml_images"):
            if image_name.endswith(('.png', '.jpg', '.jpeg')):
                img = cv2.imread(f"{os.path.dirname(os.path.abspath(__file__))}/xml_images/{image_name}")
                green_mask = cv2.inRange(img, (0, 255, 0), (0, 255, 0))
                for contour in cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                    x, y, w, h = cv2.boundingRect(contour)
                    # Escribir los valores separados por comas
                    f.write(f"{image_name},{x},{y},{x + w},{y + h}\n")
    time.sleep(1)
    print('Make xml data')

def add_coordinates():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    combined_folder = os.path.join(base_dir, "combined")
    xml_images_folder = os.path.join(base_dir, "data")
    coordinates_txt_path = os.path.join(xml_images_folder, "coordinates.txt")
    aji_txt_path = os.path.join(xml_images_folder, "val_to_xmls.txt")
    if not os.path.exists(combined_folder):
        print(f"Error: The folder '{combined_folder}' does not exist.")
        return
    images = [f for f in os.listdir(combined_folder) if f.endswith('.png')]
    coordinates = {
        parts[0].replace('.png', ''): parts[1:]
        for parts in (line.strip().split(',') for line in open(coordinates_txt_path, 'r') if line.strip())
        if len(parts) > 1
    } if os.path.exists(coordinates_txt_path) else {}
    with open(aji_txt_path, 'w') as file:
        for image in images:
            base_name = image.replace('.png', '')
            match = re.match(r"^(\d+(_\d+)?)_([a-zA-Z])$", base_name)
            if match:
                key = match.group(1)
                if key in coordinates:
                    file.write(f"{image},{','.join(coordinates[key])}\n")
                    continue
            file.write(f"{image}\n")
    time.sleep(1)
    print("Added coordinates to pictures")

def dev_xmls():
    global name_obj  # Usar la variable global
    name_obj = input("Name for object?: ")  # Asignar valor a la variable global
    base_path = os.path.dirname(os.path.abspath(__file__))
    os.makedirs(f"{base_path}/xmls", exist_ok=True)

    with open(f"{base_path}/data/val_to_xmls.txt", "r") as f:
        for linea in f:
            parts = linea.strip().split(",")
            image_filename = parts[0]
            xmin, ymin, xmax, ymax = map(int, parts[1:])
            xml_content = f"""<annotation>
    <filename>{image_filename}</filename>
    <folder>{name_obj}</folder>
    <source>
        <database>{name_obj}</database>
        <annotation>custom</annotation>
        <image>custom</image>
    </source>
    <size>
        <width>640</width>
        <height>480</height>
        <depth>3</depth>
    </size>
    <segmented>0</segmented>
    <object>
        <name>{name_obj}</name>
        <pose>unspecified</pose>
        <truncated>0</truncated>
        <difficult>0</difficult>
        <bndbox>
            <xmin>{xmin}</xmin>
            <ymin>{ymin}</ymin>
            <xmax>{xmax}</xmax>
            <ymax>{ymax}</ymax>
        </bndbox>
    </object>
</annotation>"""
            with open(f"{base_path}/xmls/{image_filename.split('.')[0]}.xml", "w") as xml_file:
                xml_file.write(xml_content)
    time.sleep(1)
    print('Making XMLS')

def structure_decnet():
    global name_obj
      # Usar la variable global
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), name_obj)
    os.makedirs(base, exist_ok=True)
    
    # Crear subcarpetas y archivo label.txt
    for sub in ["Annotations", "ImageSets", "JPEGImages"]:
        os.makedirs(os.path.join(base, sub), exist_ok=True)
    with open(os.path.join(base, "labels.txt"), "w") as f:
        f.write(name_obj)
    
    # Mover imágenes de 'combined' a 'JPEGImages'
    combined_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "combined")
    if os.path.exists(combined_folder):
        for file in os.listdir(combined_folder):
            if file.endswith(('.jpg', '.jpeg', '.png')):
                shutil.move(os.path.join(combined_folder, file), os.path.join(base, "JPEGImages", file))
    
    # Mover archivos XML de 'xmls' a 'Annotations'
    base_dir = os.path.dirname(os.path.abspath(__file__))
    xml_folder = os.path.join(base_dir, "xmls")
    annotations_folder = os.path.join(base, "Annotations")
    if os.path.exists(xml_folder):
        for file in os.listdir(xml_folder):
            if file.endswith('.xml'):
                shutil.move(os.path.join(xml_folder, file), os.path.join(annotations_folder, file))

    # Leer nombres de las imágenes en 'JPEGImages' (sin extensiones)
    jpeg_images_folder = os.path.join(base, "JPEGImages")
    image_names = [os.path.splitext(file)[0] for file in os.listdir(jpeg_images_folder) if file.endswith(('.jpg', '.jpeg', '.png'))]
    
    # Crear carpeta 'Main' dentro de 'ImageSets' y archivos .txt
    imagesets_folder = os.path.join(base, "ImageSets", "Main")
    os.makedirs(imagesets_folder, exist_ok=True)
    
    # Crear archivos .txt con los nombres de las imágenes sin extensión
    for filename in ["test.txt", "train.txt", "trainval.txt", "val.txt"]:
        with open(os.path.join(imagesets_folder, filename), "w") as f:
            for name in image_names:
                f.write(name + "\n")
    time.sleep(1)
    print("Strucure created and Data moved inside")

def move_folder():
    global name_obj
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), name_obj)
    dst = os.path.join(os.path.expanduser("~/jetson-inference/python/training/detection/ssd/data/"), name_obj)
    print(f"Source path: {src}")
    print(f"Destination path: {dst}")
    if os.path.exists(src): shutil.move(src, dst)
    time.sleep(1)
    print("Folder Moved")

def clear_folders():
    for folder in ["isolated", "combined", "xml_images", "data", "xmls"]:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), folder)
        if os.path.exists(path):
            [os.unlink(p) for p in (os.path.join(path, f) for f in os.listdir(path)) if os.path.isfile(p)]
    time.sleep(1)
    print("Cleaning Folders for next generation")
    
    
def start_training():
    global name_obj
    print(f"name of object: {name_obj}")
    path = os.getcwd()
    print(path)
    jetson_inference_path = str(Path.home()) + "/jetson-inference"
    os.chdir(jetson_inference_path)
    mi_path = jetson_inference_path 
    os.environ['MI_PATH'] = mi_path 
    subprocess.run(['docker/run.sh', '--volume', f'{path}:/custom', '-r', '/custom/start_custom_training.sh', name_obj], check=True)


#### DESCRIPTION ####
# capture_background()### 1 ### TAKES PICTURES OF THE BACKGROUND
# capture_object_cam_trackbars()### 2 ### CAPTURES THE OBJECT
# rotate_images()### 3 ### Image rotation without background
# combine_images()### 4 ### Applies backgrounds to images
# draw_rectangle()### 5 ### Draws where the object is located
# rotate_xml_images()### 6 ### Rotates the images with the green square to match the combined images
# get_coordinates()### 7 ### Extracts the information from the green boxes in the xml_images to place them in the xml of the combined pictures
# add_coordinates()### 8 ### Uses coordinates to place them in each combined picture
# dev_xmls()### 9 ### Creates xmls from val_to_xmls
# structure_decnet()### 10 ### Creates the structure for the Jetson folder
# move_folder() ### 11 ### Moves the created object folder to Jetson
# clear_folders() ### Clears the folders for the next training
# start_training() ### Runs the script that sends you to Docker and starts the training
#####################

functions = [
    capture_background,
    capture_object_cam_trackbars,
    rotate_images,
    combine_images,
    draw_rectangle,
    rotate_xml_images,
    get_coordinates,
    add_coordinates,
    dev_xmls,
    structure_decnet,
    move_folder,
    clear_folders

]

def main():
    for func in tqdm(functions, desc="Progress", ncols=100):
        func()

if __name__ == "__main__":
    main()
    print("Start Training")
    start_training()
