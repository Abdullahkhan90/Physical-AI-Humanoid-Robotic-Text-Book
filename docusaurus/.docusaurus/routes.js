import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-textbook-physical-ai-humanoid/__docusaurus/debug',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/__docusaurus/debug', '3e3'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/__docusaurus/debug/config',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/__docusaurus/debug/config', '0e6'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/__docusaurus/debug/content',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/__docusaurus/debug/content', 'c4c'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/__docusaurus/debug/globalData', '543'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/__docusaurus/debug/metadata', 'b98'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/__docusaurus/debug/registry', '023'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/__docusaurus/debug/routes', 'f4d'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/module/',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/module/', 'd37'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/module/content',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/module/content', 'ab6'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/modules',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/modules', '06d'),
    exact: true
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/docs',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs', 'b88'),
    routes: [
      {
        path: '/ai-textbook-physical-ai-humanoid/docs',
        component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs', 'e4e'),
        routes: [
          {
            path: '/ai-textbook-physical-ai-humanoid/docs',
            component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs', '2a8'),
            routes: [
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/assets/citations',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/assets/citations', '76f'),
                exact: true
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/intro',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/intro', '120'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/integrating-python-agents',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/integrating-python-agents', '9f8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/intro',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/intro', '444'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/nodes-topics-services',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/nodes-topics-services', 'cab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/summary-exercises',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/summary-exercises', '6d9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/urdf-humanoids',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-1-ros2/urdf-humanoids', '46c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-2-digital-twin/gazebo-physics',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-2-digital-twin/gazebo-physics', '898'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-2-digital-twin/intro',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-2-digital-twin/intro', 'bd7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-2-digital-twin/unity-integration',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-2-digital-twin/unity-integration', 'bf4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/intro',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/intro', '77a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/isaac-sim',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/isaac-sim', 'd71'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/nav2-humanoid',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/nav2-humanoid', 'dec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/visual-slam',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-3-ai-brain/visual-slam', '1c3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-4-vla/capstone-project',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-4-vla/capstone-project', '37f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-4-vla/cognitive-planning',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-4-vla/cognitive-planning', 'e2d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-4-vla/intro',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-4-vla/intro', '82f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai-textbook-physical-ai-humanoid/docs/module-4-vla/voice-to-action',
                component: ComponentCreator('/ai-textbook-physical-ai-humanoid/docs/module-4-vla/voice-to-action', '694'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ai-textbook-physical-ai-humanoid/',
    component: ComponentCreator('/ai-textbook-physical-ai-humanoid/', '175'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
