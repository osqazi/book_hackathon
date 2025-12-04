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
        type: 'generated-index',
        title: 'ROS 2 Fundamentals',
        description: 'Learn the foundation of humanoid robot control with ROS 2',
      },
      items: [
        // Will be populated with module-1-ros2 pages
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation & Digital Twins',
      link: {
        type: 'generated-index',
        title: 'Simulation & Digital Twins',
        description: 'Master physics simulation with Gazebo and Unity',
      },
      items: [
        // Will be populated with module-2-simulation pages
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Driven Perception (NVIDIA Isaac)',
      link: {
        type: 'generated-index',
        title: 'AI-Driven Perception',
        description: 'Advanced perception and navigation with NVIDIA Isaac',
      },
      items: [
        // Will be populated with module-3-isaac pages
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'generated-index',
        title: 'Vision-Language-Action Integration',
        description: 'Build autonomous humanoid systems with LLMs and multimodal perception',
      },
      items: [
        // Will be populated with module-4-vla pages
      ],
    },
    'references',
  ],
};

module.exports = sidebars;
