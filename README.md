# PROJET ROBOTIQUE MTBA6 2022 GROUPE 55

Projet fonctionnant avec le robot e-puck 2 : https://www.gctronic.com/doc/index.php/e-puck2
![photo_proj_4](https://user-images.githubusercontent.com/103629597/168483087-b0c9b518-987e-4f71-b2bc-e28a2ddbc361.jpg = 500x500)


Utilisation : Scan une couleur via la caméra du robot, puis se déplace sur un cercle de couleur pour arriver sur la couleur scannée. Le cercle a un rayon de 37.5 cm et le robot se place en son centre à l'initialisation. (Source du cercle : https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:Color_circle_(RGB).svg). Pour commencer à capturer l'image, il faut appuyer sur le bouton "User" du robot. Il se déplace au bout de quelques secondes sur la couleur correspondante. Il enregistre la position à laquelle il est, il n'y a donc pas besoin de replacer le robot au centre du cercle. Il peut y avoir une petite erreur en fonction de la lumière ambiante. De plus en fonction du terrain sur lequel le robot se déplace, il peut y avoir du glissement quand le robot s'arrête ou se déplace, se qui peut créer une erreur au niveau de sa position.

L'environnement sur lequel le projet a été programmé est Eclipse_e-puck2.

Debugging: Une fonction de debugging est disponible pour déterminer la valeur RGB déterminée par la caméra. Un script Python permet d'afficher les valeurs RGB fournies par le robot.
