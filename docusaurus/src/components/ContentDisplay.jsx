import React from 'react';
import styles from './ContentDisplay.module.css';

const ContentDisplay = ({ content }) => {
  if (!content) {
    return (
      <div className={styles.contentDisplay}>
        <p>No content available.</p>
      </div>
    );
  }

  return (
    <div className={styles.contentDisplay}>
      <header className={styles.contentHeader}>
        <h1 className={styles.contentTitle}>{content.title}</h1>
        <div className={styles.contentMeta}>
          <span className={styles.wordCount}>Word Count: {content.wordCount}</span>
          <span className={styles.readingTime}>Reading Time: ~{content.estimatedReadingTime} min</span>
          <span className={styles.contentType}>Type: {content.contentType}</span>
        </div>
      </header>
      
      <div className={styles.contentBody}>
        {/* In a real implementation, this would render markdown content */}
        <div 
          className={styles.contentText}
          dangerouslySetInnerHTML={{ __html: content.content }} 
        />
      </div>
      
      {content.requiredCitations && content.requiredCitations.length > 0 && (
        <div className={styles.citationsSection}>
          <h3>Required Citations</h3>
          <ul className={styles.citationsList}>
            {content.requiredCitations.map((citation, index) => (
              <li key={index} className={styles.citationItem}>
                <span className={styles.citationText}>{citation.apaFormatted}</span>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};

export default ContentDisplay;