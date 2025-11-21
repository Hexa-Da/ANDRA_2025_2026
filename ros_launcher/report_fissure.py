from PIL import Image
import matplotlib.pyplot as plt
import yaml
import os
from datetime import datetime

def tracer_point(points):
    # Charger le fichier YAML
    yaml_path = "wall_techlab.yaml"
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    # Extraire les infos nécessaires
    pgm_path = data['image']
    resolution = data['resolution']
    origin = data['origin'][:2]  # On ne garde que x et y

    # Résoudre le chemin relatif de l'image si nécessaire
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    # Charger l'image
    img = Image.open(pgm_path)
    width, height = img.size

    # Tracer l'image et les points
    plt.figure()
    plt.imshow(img, cmap='gray', origin='upper')
    point = points
    pixel_x = (point[0] - origin[0]) / resolution
    pixel_y = (height - 1) - ((point[1] - origin[1]) / resolution)  # Inversion verticale
    plt.scatter(pixel_x, pixel_y, c='red', s=50, label=f'{point}')

    plt.axis('off')
    plt.title("Carte + Points")

    # Supprimer doublons de légende
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())

    # Générer un nom de fichier basé sur la date et l'heure
    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    output_filename = f"map_with_points_{timestamp}.png"

    # Sauvegarder l'image
    plt.savefig(output_filename, bbox_inches='tight', pad_inches=0.1)
    print(f"Image enregistrée sous : {output_filename}")

    plt.show()











