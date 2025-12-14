import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Explore Textbook Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="AI Native Textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Module 1: The Robotic Nervous System (ROS 2)</h3>
                <p>Learn about ROS 2 fundamentals, nodes, topics, services, and actions that form the backbone of robotic communication.</p>
              </div>
              <div className="col col--4">
                <h3>Module 2: The Digital Twin (Gazebo & Unity)</h3>
                <p>Create realistic simulation environments using Gazebo and Unity to develop and test robots safely.</p>
              </div>
              <div className="col col--4">
                <h3>Module 3: The AI-Robot Brain (NVIDIA Isaac)</h3>
                <p>Develop advanced AI capabilities for your robots using NVIDIA Isaac SDK and GPU-accelerated computing.</p>
              </div>
            </div>
            <div className="row" style={{marginTop: '2rem'}}>
              <div className="col col--4 offset--2">
                <h3>Module 4: Vision-Language-Action (VLA)</h3>
                <p>Explore the cutting-edge intersection of computer vision, natural language processing, and robotic manipulation.</p>
              </div>
              <div className="col col--4">
                <h3>Real-World Applications</h3>
                <p>Apply your learning to practical humanoid robotics challenges including navigation, manipulation, and task execution.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}