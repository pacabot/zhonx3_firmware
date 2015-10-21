# Introduction #

Documentation d'utilisation de GIT.

# Détails #
installation
```bash

sudo apt-get install git-all gitg easygit
```
Configuration initiale de GIT :

```bash

git config --global user.name "Votre nom complet"
git config --global user.email votre_email@example.com
```

Créer un dépôt GIT :

```bash

mkdir repo_name-git
cd repo_name-git
```

=>Organiser le repo\_git.

Initialiser le dépôt GIT :

```bash

git init .
git add *
```

=>Créer un repo\_git sur votre googlecode.

Ajouter une branche distante sur le dépôt GIT :

```bash

git remote add origin uri_remote_repo_git
```

=>Travailler sur le repo\_git.

Faire un "commit" de son travail :

```bash

git commit -a -s
```

=>Mettre un changelog complet.

Pousser son code :

```bash

git push origin
```

=>Et votre travail est accessible pour les autres et versionné.

Récupérer ou cloner un dépôt GIT :

```bash

git clone uri_repo_git
```

la première fois

Ensuite un simple pour le mettre à jour :

```bash

git pull
```

Reprendre à "Travailler sur le repo\_git" pour participer après un clonage.

Ajouter un fichier au repo\_git :

```bash

git add /path/to/file
```

Regarder les changements non commis dans repo\_git :

```bash

git diff
```

Créer une nouvelle branche :

```bash

git checkout -b branch_name
```

Changer de branche :

```bash

git checkout branch_name
```

Consulter la liste des changements :

```bash

git log
```

Mettre dans le grenier les changements non commis :

```bash

git stash
```

et les ré-appliquer :

```bash

git stash apply
```

Fusionner une branche :

```bash

git merge branch_name
```

Gérer un conflit avec un outil de fusion à 3 directions :

```bash

git mergetool
```

Git demande le mot de passe à chaque fois, Il est possible de rentrer les mots de passe dans un fichier .netrc dans le répertoire utilisateur /home/user. Le fichier ressemble à ceci :

```bash

machine code.google.com login usernamea@gmail.com password blablabla
```

Mais si lors de la création du dépot, l'url utilisée contenait l'identiant, git demandera le mot de passe
à chaque fois. Afin de résoudre le problème, il faut se placer dans le répertoire du dépot. Puis aller dans
le sous-dossier .git et modier dans le fichier config pour enlever la partie login@ qu'il pourrait y avoir.

Si vous utilisez git gui et que vous avez ce message d'erreur au lancement :
Erreur : No word lists can be found for the language "fr\_FR".
Il faut installer un paquet aspell-fr qui contient le dictionnaire français pour aspell

```bash

sudo apt-get install aspell-fr
```