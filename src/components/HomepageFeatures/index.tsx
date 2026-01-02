import React, { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'About This Book',
    description: (
      <>
        <p>
          This book explores the fascinating world of Physical AI and Humanoid Robotics,
          covering the latest advances in artificial intelligence, robotics, and human-robot interaction.
        </p>
      </>
    ),
  },
  {
    title: 'Advanced Topics',
    description: (
      <>
        <p>
          Dive deep into topics like embodied intelligence, motor control, perception systems,
          and the integration of AI with physical systems.
        </p>
      </>
    ),
  },
  {
    title: 'Practical Applications',
    description: (
      <>
        <p>
          Learn about real-world applications of humanoid robots in healthcare, manufacturing,
          education, and service industries.
        </p>
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}