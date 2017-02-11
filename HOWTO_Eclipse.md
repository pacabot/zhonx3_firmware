# Introduction #

Comment installer votre environnement de développement Eclipse sur linux

# Détails #

## 1 - installer Java ##
```
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt install -y oracle-java8-installer
sudo apt install oracle-java8-set-default
sudo apt-get install oracle-java8-unlimited-jce-policy (obligatoire pour faire fonctionner le dernier plugin gnu arm eclipse)
```
## 2 - Installer Eclipse Neon2 C/C++ 64bits pour Linux ##
```
Telechargement : https://www.eclipse.org/downloads/download.php?file=/oomph/epp/neon/R2a/eclipse-inst-linux64.tar.gz
tar xvfz  ~/Téléchargements/eclipse-inst-linux64.tar.gz
Lancement : ~/Téléchargements/eclipse-installer/eclipse-inst 
Choisir Elipse IDE for C/C++ Developers
```
Créer exécutable :
```
gedit .local/share/applications/Eclipse.desktop
```
[Desktop Entry]
Type=Application
Name=Eclipse
Comment=Eclipse Integrated Development Environment
Icon=/home/eclipsePath/icon.xpm
Exec=eclipsePath/eclipse
Terminal=false
Categories=Development;IDE;Java;
Name[fr_FR]=Eclipse
```
chmod +x .local/share/applications/Eclipse.desktop
```
## ajoutez ces 3 lignes (au fichier vide), enregistrez puis fermez : ##
```
## 3 - Installer openocd ##
```
sudo apt-get install openocd libftdi-dev libusb-dev
sudo add-apt-repository ppa:laurent-boulard/openocd
sudo apt-get update
sudo apt-get install openocd
```
## 4 - Installer gcc ##
```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
```
## 5 - Plugin GNU ARM Eclipse ##
```
dans Eclispe :
help => Eclipse Marketplace.... => find GNU ARM Eclipse
installez le plugin
```
## 6 - Plugin Dark Eclipse (optionnel) ##
```
dans Eclispe :
help => Eclipse Marketplace.... => find Darkest Dark Theme 2017
installez le plugin
```
## 7 - External Tools Configurations ##
```
Cliquer sur l'icône en forme de flèche à droite de "External Tools" et choisir "External Tools Configuration"=>
Cliquer sur "Program" et cliquer sur l'icône "New Launch Configuration"=>
Saisir dans "Name" le nom de configuration "flash"
Saisir dans "Location" : ${openocd_path}/${openocd_executable}
Saisir dans "Working Directory" : ${workspace_loc:/${project_name}}
Saisir dans "Arguments" l'ensemble de lignes suivantes (F4x_Discovery) :
```
-f interface/stlink-v2.cfg
-f target/stm32f4x.cfg

-c "init" 

-c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase ${config_name:/${project_name}}/${project_name}.elf" -c "verify_image  ${config_name:/${project_name}}/${project_name}.elf" -c "sleep 100" -c "reset run" -c "exit"
```
Saisir dans "Arguments" l'ensemble de lignes suivantes (F4x_Nucleo) :
```
-f interface/stlink-v2-1.cfg
-f target/stm32f4x.cfg

-c "init" 

-c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase ${config_name:/${project_name}}/${project_name}.elf" -c "verify_image  ${config_name:/${project_name}}/${project_name}.elf" -c "sleep 100" -c "reset run" -c "exit"
```
Saisir dans "Arguments" l'ensemble de lignes suivantes (F7x) :
```
-f  board/stm32f7discovery.cfg

-c "init" 

-c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase ${config_name:/${project_name}}/${project_name}.elf" -c "verify_image  ${config_name:/${project_name}}/${project_name}.elf" -c "sleep 100" -c "reset run" -c "exit"
```
Dans l'onglet "Common", cliquer sur la boîte à cocher "External Tools" (Display in favorites menu)=>
Cliquer sur le bouton "Apply" puis sur "Close"
```
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
(F4x_Discovery) :
-f interface/stlink-v2.cfg
-f target/stm32f4x_stlink.cfg
```
(F4x_Nucleo) :
-f interface/stlink-v2-1.cfg
-f target/stm32f4x.cfg
```
(F7x) :
-f  board/stm32f7discovery.cfg
```
Dans la seconde partie (GDB Client Setup), remplacer le contenu de la zone "Commandes" par l'ensemble des lignes suivantes :
```
set mem inaccessible-by-default off
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4
```
Dans l'onglet "Common", cliquer sur la boîte à cocher "Debug" (Display in favorites menu)=>
Cliquer sur le bouton "Apply" puis sur "Close"
