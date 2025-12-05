/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main sidebar with four modules
  mainSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        'module-1-ros2/nodes-topics',
        'module-1-ros2/services-actions',
        'module-1-ros2/urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation & Digital Twins',
      link: {
        type: 'doc',
        id: 'module-2-simulation/index',
      },
      items: [
        'module-2-simulation/physics-principles',
        'module-2-simulation/sensors',
        'module-2-simulation/digital-twin',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Driven Perception (NVIDIA Isaac)',
      link: {
        type: 'doc',
        id: 'module-3-isaac/index',
      },
      items: [
        'module-3-isaac/isaac-sim',
        'module-3-isaac/vslam',
        'module-3-isaac/nav2',
        'module-3-isaac/synthetic-data',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'module-4-vla/index',
      },
      items: [
        'module-4-vla/llm-planning',
        'module-4-vla/whisper',
        'module-4-vla/multimodal',
        'module-4-vla/architecture',
      ],
    },
    'references',
  ],
};

module.exports = sidebars;
