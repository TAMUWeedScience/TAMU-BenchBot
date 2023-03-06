# BenchBot

### Table of Contents
**[Clone this repo](#clone-this-repo)**<br>
**[Install Miniconda (forge)](#install-miniconda-forge)**<br>
**[Install Conda environment](#install-conda-environment)**<br>
**[Run image collection](#run-image-collection)**<br>
**[Troubleshooting](#troubleshooting)**<br>
**[Background](#background)**<br>

---

Setup and installation of software require internet conection

## Project setup

1. Make sure you're in the home directory

    `cd ~`

2. Clone this repo

    `git clone https://github.com/precision-sustainable-ag/BenchBot-SonyHRCam.git`

## Install Miniconda (forge)

1. Move into the Downloads folder

    `cd ~/Downloads`

2. Download installer file
  
    `wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh`

3. Run the installer

    `bash Miniforge3-Linux-aarch64.sh`

    * Agree to terms, press "d" to scroll down the license terms page, and accept license terms.

    * Press `ENTER` to confirm location which should be `home/pi/miniforge3`
    * Type `yes` when the installer asks, "Do you wish the installer to initialize Miniforge3 by running conda init?"

4. Avoid activating the base environment on startup
    
    `conda config --set auto_activate_base false`

5. Update conda

    `conda update -n base -c conda-forge conda`

--- 

## Create the conda environment

1. Move into the project repo

    `cd ~/BenchBot-SonyHRCam`

2. Create the conda environment

    `conda env create -f environment_rpi.yml`

      * press `y` to proceed download.

3. Activate the environment

    `conda activate bbot`

---

## Camera Settings

### PC Remote Connectoin

Change in camera settings. Go to `menu -> network -> 1/3 -> PC Remote Function -> PC Remote`. Choose `On`. 

### File Format

Change in camera settings. Go to `menu -> camera1 -> 1/15 -> File Format`. Choose `RAW & JPEG`

### RAW File Type

Change in camera settings. Go to `menu -> camera1 -> 1/15 -> RAW File Type`. Choose `Compressed`

### F-Number 

Change by rotating dial nearest to camera trigger button.

Depending on outside lighting, F number should be betwee 10-13. The darker it is outside the lower the number. For example, on days with very overcast skies, use F10; on days with full sun, use F13.

### Focus area

Change in camera settings. Go to `menu -> camera1 -> 5/15 -> Focus Area Limit`. Only `Wide` and `Zone` should be selected.

## Run image collection


## Troubleshooting

### Permission denied `support/SONY_rpi/RemoteCli` 

From main repo directory, give executable permission for RemoteCli. Type:

`chmod +x support/Sony_rpi/RemoteCli`

### Camera power supply issues

Camera default settings use the USB as a power supply. This may cause connection/power issues. To change this, go to `menu -> Setup -> 4/7 -> USB Popwer Supply`. Switch to `Off`.

## Helpful Commands

### Check python version

`python -V `

### Check conda version

`conda -V`

### List the created conda environments

`conda env list`

### List all install packages

`pip list`

### Update current environment (bbot) from changes in yaml file.
 
`conda env update --name bbot --file local.yml --prune`


<br>

---

<br>
<br>

# Background

<p align="center">
  <img src= "https://user-images.githubusercontent.com/45602572/186969821-aef45304-8a6a-4775-a404-9dbfcef3045f.jpg" width="700">
</p> 

High-throughput technology for plant phenotyping allows plant development and morphological traits tracking at a scale that was unimaginable with traditional phenotyping techniques. The development of  high-throughput phenotyping tools has been exponentially growing for several years now thanks to the increasing availability of lower cost sensors and technological advances in general. There are numerous examples of these tools which are being used for different applications such as root, shoot, and tissue phenotyping.
Even though there are several options out there, most of them are either costly, need specific infrastructure, are limited to certain plant sizes or require large amounts of highly trained labor to operate. This is the gap that we are expecting to cover by developing a a versatile low-cost high-throughput tool such as BenchBot. 
BenchBot is an open-source high-throughput plant phenotyping computer vision solution which allows for data collection in an automated way, it can be used in different setups (indoor/semi-outdoor) and with various sensors. It is relatively low-cost, modular, upgradeable, portable and easy to use.
BenchBot can be used with various sensors to adjust to the application needs. In this repository you can find the scripts being used for the integration and control of BenchBot with a High Resolution camera. bla ba bla add here. 
For a full description of BenchBot, list of materials, camera specs, among others, visit the [BenchBot Confluence](https://precision-sustainable-ag.atlassian.net/l/cp/eeUgH9Bm) page.