// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI Native Textbook on Physical AI & Humanoid Robotics',
  tagline: 'Teaching the principles of Physical AI and Humanoid Robotics through hands-on experiences',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://abdullahkhan90.github.io', // Updated to match your GitHub username
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages: https://<USERNAME>.github.io/<REPO>/
  baseUrl: '/Physical-AI-Humanoid-Robotic-Text-Book/', // Updated to match your repository name

  // GitHub pages deployment config.
  organizationName: 'Abdullahkhan90', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotic-Text-Book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'], // Temporarily disabling 'ur' due to translation file issues
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/edit/main/',
        },
        blog: {
          showReadingTime: true,
          path: './blog',
          routeBasePath: '/blog',
          // Please change this to your repo.
          editUrl:
            'https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book/edit/main/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      metadata: [
        {name: 'description', content: 'AI-Native Textbook on Physical AI & Humanoid Robotics'},
        {name: 'keywords', content: 'robotics, AI, textbook, humanoid, physical AI'}
      ],
      navbar: {
        title: 'Physical AI Humanoid Robotic Text Book',
        logo: {
          alt: 'Physical AI Humanoid Robotic Text Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Textbook Modules',
          },
          { to: '/modules', label: 'Updates', position: 'left' },
          {
            href: 'https://www.linkedin.com/in/hafiz-abdullah-4239a62a4',
            label: 'LinkedIn',
            position: 'right',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: The Robotic Nervous System (ROS 2)',
                to: '/docs/module-1-ros2/intro',
              },
              {
                label: 'Module 2: The Digital Twin (Gazebo & Unity)',
                to: '/docs/module-2-digital-twin/intro',
              },
              {
                label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
                to: '/docs/module-3-ai-brain/intro',
              },
              {
                label: 'Module 4: Vision-Language-Action (VLA)',
                to: '/docs/module-4-vla/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discord',
                href: 'https://discord.gg/physical-ai',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/physical_ai',
              },
              {
                label: 'LinkedIn',
                href: 'https://linkedin.com/company/physical-ai-community',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Abdullahkhan90/Physical-AI-Humanoid-Robotic-Text-Book',
              }
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI Native Textbook on Physical AI & Humanoid Robotics Project. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;