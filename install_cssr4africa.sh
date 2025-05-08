#!/bin/bash

# CSSR4Africa Installation Script
# This script installs the CSSR4Africa software and models for both physical and simulator robots
# Usage: chmod +x install_cssr4africa.sh && ./install_cssr4africa.sh

echo "===== CSSR4Africa Installation Script ====="
echo ""

# Function to install for physical robot
install_physical_robot() {
    echo "Installing for Physical Robot..."
    
    # 1. Clone and Build the Software
    echo "1. Cloning and building the software..."
    cd $HOME/workspace/pepper_rob_ws/src && \
    git clone https://github.com/cssr4africa/cssr4africa.git && \
    cd $HOME/workspace/pepper_rob_ws && catkin_make
    
    # 2. Clone the Models from HuggingFace
    echo "2. Cloning models from HuggingFace..."
    cd ~
    git lfs install
    git clone https://huggingface.co/cssr4africa/cssr4africa_models
    
    # 3. Move the Face Detection Models
    echo "3. Moving face detection models..."
    mkdir -p $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/models
    mv ~/cssr4africa_models/face_detection/models/* \
      $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/models/
    
    # 4. Move the Person Detection Models
    echo "4. Moving person detection models..."
    mkdir -p $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/models
    mv ~/cssr4africa_models/person_detection/models/* \
      $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/person_detection/models/
    
    # 5. Clone the Unit Test Data from HuggingFace
    echo "5. Cloning unit test data from HuggingFace..."
    cd ~
    git lfs install
    git clone https://huggingface.co/cssr4africa/cssr4africa_unit_tests_data_files
    
    # 6. Move the Face Detection Test Data
    echo "6. Moving face detection test data..."
    mkdir -p $HOME/workspace/pepper_rob_ws/src/unit_tests/face_detection_test/data
    mv ~/cssr4africa_unit_tests_data_files/face_detection_test/data/* \
      $HOME/workspace/pepper_rob_ws/src/unit_tests/face_detection_test/data/
    
    # 7. Move the Person Detection Test Data
    echo "7. Moving person detection test data..."
    mkdir -p $HOME/workspace/pepper_rob_ws/src/unit_tests/person_detection_test/data
    mv ~/cssr4africa_unit_tests_data_files/person_detection_test/data/* \
      $HOME/workspace/pepper_rob_ws/src/unit_tests/person_detection_test/data/
      
    echo "Physical robot installation completed!"
}

# Function to install for simulator robot
install_simulator_robot() {
    echo "Installing for Simulator Robot..."
    
    # 1. Clone and Build the Software
    echo "1. Cloning and building the software..."
    cd $HOME/workspace/pepper_sim_ws/src && \
    git clone https://github.com/cssr4africa/cssr4africa.git && \
    cd $HOME/workspace/pepper_sim_ws && catkin_make -DSIMULATOR=ON
    
    echo "Simulator robot installation completed!"
}

# Main installation process
echo "What would you like to install?"
echo "1) Physical Robot"
echo "2) Simulator Robot"
echo "3) Both"
read -p "Enter your choice (1-3): " choice

case $choice in
    1)
        install_physical_robot
        ;;
    2)
        install_simulator_robot
        ;;
    3)
        install_physical_robot
        install_simulator_robot
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "===== Installation Complete ====="
echo "You can verify the installation by running the unit tests:"