import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  bookSidebar: [
    {
      type: 'category',
      label: 'Book',
      items: [
        'introduction', // Assuming an existing introduction.md
        {
          type: 'category',
          label: 'Module 1: Foundations of Physical AI',
          items: [
            'module1-introduction', // Added module introduction
            'module1-chapter1-middleware',
            'module1-chapter2-ros2-core-concepts',
            'module1-chapter3-rclpy-bridge',
            'module1-chapter4-urdf-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            'module2-introduction', // The new introduction for Module 2
            'module2-chapter1-physics-simulation', // Assuming a file for this chapter
            'module2-chapter2-gazebo-physics', // Assuming a file for this chapter
            'module2-chapter3-unity-rendering', // Assuming a file for this chapter
            'module2-chapter4-simulating-sensors', // The chapter I just created
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'module3-introduction',
            'module3-chapter1-advanced-perception-and-training',
            'module3-chapter2-nvidia-isaac-sim',
            'module3-chapter3-isaac-ros',
            'module3-chapter4-nav2-path-planning',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          items: [
            'module4-introduction',
            'module4-chapter1-llms-and-robotics',
            'module4-chapter2-voice-to-action',
            'module4-chapter3-cognitive-planning',
            'module4-chapter4-capstone-project',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
