import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './ModuleNavigation.module.css';

const ModuleNavigation = ({ modules }) => {
  const { siteConfig } = useDocusaurusContext();
  const [activeModule, setActiveModule] = useState(null);

  useEffect(() => {
    // Set the first module as active by default
    if (modules && modules.length > 0) {
      setActiveModule(modules[0].id);
    }
  }, [modules]);

  return (
    <div className={styles.moduleNavigation}>
      <h2>{siteConfig.title} Modules</h2>
      <div className={styles.moduleGrid}>
        {modules.map((module) => (
          <div 
            key={module.id} 
            className={`${styles.moduleCard} ${activeModule === module.id ? styles.active : ''}`}
            onClick={() => setActiveModule(module.id)}
          >
            <h3 className={styles.moduleTitle}>
              <Link to={module.contentPath}>
                {module.title}
              </Link>
            </h3>
            <p className={styles.moduleDescription}>{module.description}</p>
            <div className={styles.moduleTopics}>
              <h4>Topics:</h4>
              <ul>
                {module.topics.slice(0, 3).map((topic, index) => (
                  <li key={index} className={styles.topicItem}>{topic}</li>
                ))}
                {module.topics.length > 3 && (
                  <li className={styles.topicItem}>+ {module.topics.length - 3} more</li>
                )}
              </ul>
            </div>
            <div className={styles.moduleLearningObjectives}>
              <h4>Learning Objectives:</h4>
              <ul>
                {module.learningObjectives.slice(0, 2).map((objective, index) => (
                  <li key={index} className={styles.objectiveItem}>{objective}</li>
                ))}
                {module.learningObjectives.length > 2 && (
                  <li className={styles.objectiveItem}>View all objectives...</li>
                )}
              </ul>
            </div>
            <div className={styles.moduleStatus}>
              <span className={`${styles.statusBadge} ${module.isActive ? styles.activeStatus : styles.inactiveStatus}`}>
                {module.isActive ? 'Active' : 'Inactive'}
              </span>
              <span className={styles.moduleOrder}>Order: {module.order}</span>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default ModuleNavigation;