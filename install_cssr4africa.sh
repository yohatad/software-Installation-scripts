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

# Function to install face and person detection environment
install_face_person_detection_environment() {
    echo "Setting up Face and Person Detection Environment..."
    
    # Update system packages
    echo "1. Updating system packages..."
    sudo apt update && sudo apt upgrade -y
    
    # Add the deadsnakes PPA for Python versions
    echo "2. Adding Python repository..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository ppa:deadsnakes/ppa -y
    sudo apt update
    
    # Install Python 3.10
    echo "3. Installing Python 3.10..."
    sudo apt install python3.10 python3.10-venv python3.10-distutils -y
    
    # Verify Python installation
    python3.10 --version
    
    # Set Up Virtual Environment
    echo "4. Setting up virtual environment..."
    mkdir -p $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs
    cd $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs
    python3.10 -m venv cssr4africa_face_person_detection_env
    
    # Activate the virtual environment
    echo "5. Activating virtual environment..."
    source cssr4africa_face_person_detection_env/bin/activate
    
    # Upgrade pip in the virtual environment
    pip install --upgrade pip
    
    # Install PyTorch with CUDA support
    echo "6. Installing PyTorch with CUDA support..."
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
    
    # Install additional requirements
    echo "7. Installing additional requirements..."
    pip install -r $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/face_detection_requirements_x86.txt
    
    echo "Face and Person Detection Environment setup completed!"
}

# Function to install sound detection environment
install_sound_detection_environment() {
    echo "Setting up Sound Detection Environment..."
    
    # Update system packages (if not already done)
    echo "1. Updating system packages..."
    sudo apt update && sudo apt upgrade -y
    
    # Install Python virtual environment tools
    echo "2. Installing Python 3.8 virtual environment tools..."
    sudo apt install python3.8-venv -y
    
    # Set Up Virtual Environment
    echo "3. Setting up sound detection virtual environment..."
    mkdir -p $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs
    cd $HOME/workspace/pepper_rob_ws/src/cssr4africa_virtual_envs
    python3.8 -m venv cssr4africa_sound_detection_env
    
    # Activate the virtual environment
    echo "4. Activating virtual environment..."
    source cssr4africa_sound_detection_env/bin/activate
    
    # Upgrade pip in the virtual environment
    pip install --upgrade pip
    
    # Install additional requirements
    echo "5. Installing sound detection requirements..."
    pip install -r $HOME/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/sound_detection/sound_detection_requirements.txt
    
    echo "Sound Detection Environment setup completed!"
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
        install_face_person_detection_environment
        install_sound_detection_environment
        ;;
    2)
        install_simulator_robot
        ;;
    3)
        install_physical_robot
        install_simulator_robot
        install_face_person_detection_environment
        install_sound_detection_environment
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "===== Installation Complete ====="
echo "You can verify the installation by running the unit tests:"