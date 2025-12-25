# AI Native Textbook on Physical AI & Humanoid Robotics

Deploy to GitHub Pages [![Build Validation](https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/actions/workflows/main.yml/badge.svg)](https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/actions/workflows/main.yml)

## Overview

This repository contains an AI Native Textbook on Physical AI & Humanoid Robotics, built with Docusaurus. The textbook covers essential topics in robotics, AI, and their integration for developing advanced humanoid robotic systems.

## Table of Contents

- [Overview](#overview)
- [Modules](#modules)
- [Features](#features)
- [Getting Started](#getting-started)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Modules

The textbook is organized into the following modules:

- **Module 1**: The Robotic Nervous System (ROS 2)
- **Module 2**: The Digital Twin (Gazebo & Unity)
- **Module 3**: The AI-Robot Brain (NVIDIA Isaac)
- **Module 4**: Vision-Language-Action (VLA)

## Features

- Interactive textbook with hands-on examples
- Comprehensive coverage of Physical AI and Humanoid Robotics
- Integration with ROS 2 for robotic nervous systems
- Physics simulation and environment building using Gazebo and Unity
- NVIDIA Isaac integration for AI-driven robotics
- Vision-Language-Action (VLA) systems for advanced robot control

## Getting Started

To run this project locally:

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book.git
   cd Physical-AI-Humanoid-Robotic-Text-Book
   ```

2. **Navigate to the docusaurus directory:**
   ```bash
   cd docusaurus
   ```

3. **Install dependencies:**
   ```bash
   npm install
   ```

4. **Start the development server:**
   ```bash
   npm run start
   ```

5. **Open your browser to [http://localhost:3000](http://localhost:3000) to see the textbook.**

## Alternative Method

From the root directory, you can use the scripts defined in the root package.json:
```bash
npm install  # Install root dependencies
npm run dev  # Start the docusaurus development server
```

## Project Structure

- `docusaurus/` - Contains the Docusaurus website source files
- `docs/` - Contains the built site for GitHub Pages
- `.github/workflows/` - Contains GitHub Actions workflows for deployment

## Contributing

We welcome contributions to improve the textbook! Please feel free to submit pull requests or create issues for suggestions and bug reports.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.