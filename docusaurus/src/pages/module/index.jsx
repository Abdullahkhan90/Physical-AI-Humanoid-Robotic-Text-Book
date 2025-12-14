import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';

export default function ModuleIndexPage() {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    <Layout
      title={`Module Content | ${siteConfig.title}`}
      description="Select a module to view its content">
      <main>
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--12">
              <header className="hero hero--primary">
                <div className="container">
                  <h1 className="hero__title">Module Content</h1>
                  <p className="hero__subtitle">Select a module to view its content</p>
                </div>
              </header>
              
              <div className="margin-vert--lg text--center">
                <p>Browse content by module:</p>
                <div className="button-group button-group--block">
                  <Link className="button button--primary" href="/module/content?module=ros2">
                    ROS 2 Content
                  </Link>
                  <Link className="button button--secondary" href="/module/content?module=gazebo">
                    Gazebo Content
                  </Link>
                  <Link className="button button--secondary" href="/module/content?module=isaac">
                    Isaac Content
                  </Link>
                </div>
                <p className="margin-top--lg">
                  <Link to="/modules">Back to all modules</Link>
                </p>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}