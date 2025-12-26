
# Physical AI & Humanoid Robotics Textbook

> Comprehensive 13-week textbook for industry practitioners: ROS 2, Digital Twin (Gazebo/Unity), NVIDIA Isaac Sim, and Vision-Language-Action models.

[![Deploy to GitHub Pages](https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/actions/workflows/deploy.yml/badge.svg)](https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/actions/workflows/deploy.yml)
(https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/actions/workflows/main.yml/badge.svg)](https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/actions/workflows/main.yml)
 https://abdullahkhan90.github.io/Physical-AI-Humanoid-Robotic-Text-Book/

## Overview

This textbook provides hands-on training for building autonomous humanoid robots using:
- **ROS 2** (Weeks 3-5): Robot Operating System fundamentals
- **Digital Twin** (Weeks 6-7): Gazebo and Unity simulation
- **NVIDIA Isaac Sim** (Weeks 8-10): GPU-accelerated simulation and synthetic data
- **Vision-Language-Action Models** (Weeks 11-13): Multimodal AI for humanoid control

**Target Audience**: Industry practitioners with Python programming knowledge, transitioning to robotics and embodied AI.

This repository contains an AI Native Textbook on Physical AI & Humanoid Robotics, built with Docusaurus. The textbook covers essential topics in robotics, AI, and their integration for developing advanced humanoid robotic systems.

## Course Structure

| Module | Weeks | Focus |
|--------|-------|-------|
| Introduction | 1-2 | Physical AI Foundations |
| Module 1: ROS 2 | 3-5 | Architecture, Topics, URDF |
| Module 2: Digital Twin | 6-7 | Gazebo, Unity, Sim2Real |
| Module 3: NVIDIA Isaac | 8-10 | Isaac Sim, Synthetic Data, Imitation Learning |
| Module 4: VLA & Humanoids | 11-13 | Multimodal Models, Transformer Policies |
| Capstone | Week 13 | Autonomous Humanoid (Voice → Action) |

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

## Hardware Paths

Choose one of three hardware configurations:

1. **Digital Twin Workstation**: RTX 3060+ GPU, Ubuntu 22.04
2. **Physical AI Edge Kit**: NVIDIA Jetson Orin Nano
3. **Cloud-Native**: AWS/Azure with GPU instances

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

## Documentation Site

This project uses [Docusaurus 3](https://docusaurus.io/) with:
- Dashboard-style homepage with module cards
- Nested sidebar with collapsible categories
- Hybrid search (Algolia + Flexsearch for glossary)
- Custom metadata for chapter prerequisites and learning objectives
- GitHub Actions CI/CD with quality gates

## Project Structure

```
Physical-AI-Humanoid-Robotics-Textbook/
├── docs/                   # Main content
│   ├── intro.md
│   ├── setup/              # 3 hardware paths
│   ├── module-1-ros2/      # Weeks 3-5
│   ├── module-2-digital-twin/   # Weeks 6-7
│   ├── module-3-isaac/     # Weeks 8-10
│   ├── module-4-vla-humanoids/  # Weeks 11-13
│   ├── capstone/
│   └── references/         # Glossary, notation, troubleshooting
├── src/                    # Custom React components
│   ├── components/
│   └── pages/index.tsx     # Dashboard homepage
├── specs/                  # Feature specifications
│   └── 001-book-master-plan/
│       ├── spec.md
│       ├── plan.md
│       ├── tasks.md
│       └── contracts/      # JSON Schema for validation
├── backend/                # Backend services for RAG chatbot
│   ├── src/
│   ├── api/
│   ├── models/
│   ├── services/
│   └── tests/
└── .github/workflows/      # CI/CD pipelines
```

## Contributing

We welcome contributions to improve the textbook! Please feel free to submit pull requests or create issues for suggestions and bug reports.

### Quality Gates

All PRs must pass:
- Build with 0 errors/warnings
- Link validation (0 broken links)
- Lighthouse scores: Performance ≥90, Accessibility ≥95, SEO ≥95
- Chapter metadata validation against JSON Schema

## Specification-Driven Development

This project follows **Spec-Driven Development (SDD)** using [Spec-Kit Plus](https://github.com/Abdullahkhan90/spec-kit-plus):

1. **Constitution**: Core principles in [`.specify/memory/constitution.md`](.specify/memory/constitution.md)
2. **Specifications**: Feature specs in [`specs/001-book-master-plan/spec.md`](specs/001-book-master-plan/spec.md)
3. **Planning**: Implementation plan in [`specs/001-book-master-plan/plan.md`](specs/001-book-master-plan/plan.md)
4. **Tasks**: Breakdown in [`specs/001-book-master-plan/tasks.md`](specs/001-book-master-plan/tasks.md)
5. **History**: Prompt History Records in [`history/prompts/`](history/prompts/)

## Technology Stack

- **Documentation**: Docusaurus 3.x
- **Language**: TypeScript 5.x
- **UI**: React 18.x
- **Backend**: Python, FastAPI
- **Database**: Qdrant (vector database)
- **Build Tools**: Node.js 18+
- **Search**: Algolia DocSearch + Flexsearch
- **CI/CD**: GitHub Actions
- **Deployment**: GitHub Pages
- **AI Assistant**: Claude Code

## Roadmap

- [ ] Week 1-2: Introduction chapters
- [ ] Week 3-5: ROS 2 module chapters
- [ ] Week 6-7: Digital Twin module chapters
- [ ] Week 8-10: NVIDIA Isaac module chapters
- [ ] Week 11-13: VLA & Humanoids module chapters
- [ ] Capstone project guide
- [ ] Assessment rubrics
- [ ] Instructor materials
- [ ] Video tutorials
- [ ] Interactive code examples

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/) by Meta
- Developed using [Claude Code](https://claude.ai/code) AI Assistant
- Following [Spec-Kit Plus](https://github.com/Abdullahkhan90/spec-kit-plus) methodology
- Inspired by industry best practices in robotics education

## Support

- **Issues**: [GitHub Issues](https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/discussions)

---

**Built with Claude Code • Powered by Spec-Driven Development**
