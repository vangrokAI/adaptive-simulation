#!/bin/bash
# Initialisiere das GitHub-Repository für das Adaptive Surface Contact Simulation Projekt
# Dieses Skript sollte im Hauptverzeichnis des Projekts ausgeführt werden

# Terminal colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Print header
echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}   GitHub Repository Initialisierung         ${NC}"
echo -e "${BLUE}   Adaptive Surface Contact Simulation       ${NC}"
echo -e "${BLUE}==============================================${NC}"

# Prüfen ob wir im richtigen Verzeichnis sind
if [ ! -d "/opt/grok/adaptive-simulation" ]; then
    echo -e "${RED}Error: Bitte führen Sie dieses Skript im Projektverzeichnis aus.${NC}"
    echo -e "${RED}       cd /opt/grok/adaptive-simulation${NC}"
    exit 1
fi

# Prüfen ob git installiert ist
if ! command -v git &> /dev/null; then
    echo -e "${RED}Git ist nicht installiert. Bitte installieren Sie git:${NC}"
    echo -e "${YELLOW}sudo apt install git${NC}"
    exit 1
fi

# Git-Repository initialisieren, falls es noch nicht existiert
if [ ! -d ".git" ]; then
    echo -e "${BLUE}Initialisiere Git-Repository...${NC}"
    git init
    if [ $? -ne 0 ]; then
        echo -e "${RED}Fehler beim Initialisieren des Git-Repositories.${NC}"
        exit 1
    fi
    echo -e "${GREEN}Git-Repository erfolgreich initialisiert.${NC}"
else
    echo -e "${YELLOW}Git-Repository existiert bereits.${NC}"
fi

# Gitignore erstellen
echo -e "${BLUE}Erstelle .gitignore-Datei...${NC}"
cat > .gitignore << EOL
# ROS2 specific ignores
install/
log/
build/

# Python specific ignores
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
*.egg-info/
.installed.cfg
*.egg

# C++ specific ignores
*.o
*.so
*.a
*.la
*.lo
*.d
*.obj
*.exe
*.out
*.app
*.gch
*.pch

# IDE specific ignores
.idea/
.vscode/
*.swp
*.swo
*~
.DS_Store

# Log files
*.log
*.stderr
*.stdout

# Backup files
*~
*#
EOL
echo -e "${GREEN}.gitignore-Datei erstellt.${NC}"

# Dateien zum Repository hinzufügen
echo -e "${BLUE}Füge Dateien zum Repository hinzu...${NC}"
git add .
if [ $? -ne 0 ]; then
    echo -e "${RED}Fehler beim Hinzufügen von Dateien zum Repository.${NC}"
    exit 1
fi
echo -e "${GREEN}Dateien erfolgreich hinzugefügt.${NC}"

# Initialen Commit erstellen
echo -e "${BLUE}Erstelle initialen Commit...${NC}"
git commit -m "Initialer Commit: Adaptive Surface Contact Simulation für Deckeninstallation"
if [ $? -ne 0 ]; then
    echo -e "${RED}Fehler beim Erstellen des initialen Commits.${NC}"
    echo -e "${YELLOW}Bitte konfigurieren Sie Ihre Git-Identität:${NC}"
    echo -e "${YELLOW}git config --global user.email \"you@example.com\"${NC}"
    echo -e "${YELLOW}git config --global user.name \"Your Name\"${NC}"
    exit 1
fi
echo -e "${GREEN}Initialer Commit erfolgreich erstellt.${NC}"

# Instruktionen für das Hinzufügen eines Remote-Repositories
echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}   Repository erfolgreich initialisiert!     ${NC}"
echo -e "${BLUE}==============================================${NC}"
echo -e "${YELLOW}Um das Repository zu einem GitHub-Remote-Repository hinzuzufügen:${NC}"
echo -e "${YELLOW}1. Erstellen Sie ein neues Repository auf GitHub${NC}"
echo -e "${YELLOW}2. Verbinden Sie das lokale Repository mit dem Remote-Repository:${NC}"
echo -e "   ${GREEN}git remote add origin https://github.com/username/adaptive-simulation.git${NC}"
echo -e "${YELLOW}3. Pushen Sie den initialen Commit:${NC}"
echo -e "   ${GREEN}git push -u origin master${NC}"
echo -e "${BLUE}==============================================${NC}"
