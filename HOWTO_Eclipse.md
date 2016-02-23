# Introduction #

Comment installer votre environnement de développement Eclipse sur linux


# Détails #

## 1 - Telecharger Eclipse Luna C/C++ 64bits pour Linux ##
http://www.eclipse.org/downloads/download.php?file=/technology/epp/downloads/release/mars/1/eclipse-cpp-mars-1-linux-gtk-x86_64.tar.gz&mirror_id=1099
## 2 - Installer ##

```
sudo tar -xvzf '/home/zhonx/Téléchargements/eclipse-cpp-luna-SR1a-linux-gtk-x86_64.tar.gz' -C /opt   //extraction dans /opt
sudo chmod -R +r /opt/eclipse                           //permission d'ecriture
```
Créer exécutable
```
sudo touch /usr/bin/eclipse
sudo chmod 755 /usr/bin/eclipse
sudo apt-get install gedit         //installation editeur gedit si vous ne l'avez pas
sudo gedit /usr/bin/eclipse
```

## ajoutez ces 3 lignes (au fichier vide), enregistrez puis fermez : ##
```
#!/bin/sh
export ECLIPSE_HOME="/opt/eclipse"
$ECLIPSE_HOME/eclipse $*
```
## 3 - Installer openocd ##
```
sudo apt-get install openocd libftdi-dev libusb-dev
```
## 4 - Installer gcc ##
```
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi
```
## 5 - Lancer Éclipse ##
```
'/opt/eclipse/eclipse'
```
## 6 - Installer plugin arm ##
dans Eclispe :
help => install new software => renseigner dans le champ "work with" l'adresse suivante "http://gnuarmeclipse.sourceforge.net/updates"
installez le plugin
## 7 - Configurer la fonction "Flash" dans Eclipse ##
Cliquer sur l'icône en forme de flèche à droite de "External Tools" et choisir "External Tools Configuration"=>
Cliquer sur "Program" et cliquer sur l'icône "New Launch Configuration"=>
Saisir dans "Name" le nom de configuration "flash"
Saisir dans "Location" : cliquer sur l'icône "Browse Filesystem" :$
```
   sélectionner le fichier "/usr/bin/openocd"
```
Saisir dans "Working Directory" l'adresse :
```
${workspace_loc/${project_name}}
```
Saisir dans "Arguments" l'ensemble de lignes suivantes :
```
-f interface/stlink-v2.cfg
-f target/stm32f4x_stlink.cfg

-c "init" 

-c "reset halt"
-c "sleep 100"
-c "wait_halt 2"
-c "flash write_image erase ${config_name:/${project_name}}/${project_name}.elf" -c "verify_image  ${config_name:/${project_name}}/${project_name}.elf" -c "sleep 100" -c "reset run" -c "exit"
```
Dans l'onglet "Common", cliquer sur la boîte à cocher "External Tools" (Display in favorites menu)=>
Cliquer sur le bouton "Apply" puis sur "Close"
## 8 - Configurer le Debug ##
Cliquer sur l'icône en forme de flèche à droite de "Debug zhonx3" et choisir "Debug Configurations"=>
Cliquer sur "GDB OpenOCD Debugging" et cliquer sur l'icône "New Launch Configuration"=>
Saisir dans "Name" le nom de configuration "Debug"=>
Saisir dans "C/C++ Application" la ligne suivante :
```
${workspace_loc:/${project_name}}/${config_name:/${project_name}/${project_name}.elf
```
Saisir dans "Project" le nom
```
${project_name}
```
Cliquer sur l'onglet "Debugger"
Dans la première partie (OpenOCD setup), saisir dans "Config options" les deux lignes suivantes =>
```
-f interface/stlink-v2.cfg
-f target/stm32f4x_stlink.cfg
```
Dans la seconde partie (GDB Client Setpup), remplacer le contenu de la zone "Commandes" par l'ensemble des lignes suivantes :
```
set mem inaccessible-by-default off
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4
```
Dans l'onglet "Common", cliquer sur la boîte à cocher "Debug" (Display in favorites menu)=>
Cliquer sur le bouton "Apply" puis sur "Close"
