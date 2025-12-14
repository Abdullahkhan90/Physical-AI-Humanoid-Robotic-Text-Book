import React, { useState, useEffect } from 'react';
import styles from './PersonalizationSettings.module.css';
import { useColorMode } from '@docusaurus/theme-common';

const PersonalizationSettings = ({ onSettingsChange }) => {
  const [language, setLanguage] = useState('en');
  const [learningPace, setLearningPace] = useState('intermediate');
  const [preferredTopics, setPreferredTopics] = useState([]);
  const [notifications, setNotifications] = useState({
    email: true,
    push: false,
  });
  const [uiSettings, setUiSettings] = useState({
    fontSize: 'medium',
    theme: 'auto',
    showAnimations: true,
  });
  const { colorMode, setColorMode } = useColorMode();

  const topics = [
    'ROS 2',
    'Gazebo Simulation',
    'NVIDIA Isaac',
    'Vision-Language-Action',
    'Machine Learning',
    'Computer Vision',
    'Robotics Control'
  ];

  const handleTopicToggle = (topic) => {
    if (preferredTopics.includes(topic)) {
      setPreferredTopics(preferredTopics.filter(t => t !== topic));
    } else {
      setPreferredTopics([...preferredTopics, topic]);
    }
  };

  const handleSave = () => {
    const settings = {
      language,
      learningPace,
      preferredTopics,
      notifications,
      uiSettings,
    };

    if (onSettingsChange) {
      onSettingsChange(settings);
    }

    // In a real implementation, this would save to the backend
    console.log('Saving personalization settings:', settings);
    alert('Settings saved successfully!');
  };

  return (
    <div className={styles.settingsContainer}>
      <h2>Learning Personalization Settings</h2>
      
      <div className={styles.settingsGrid}>
        {/* Language Settings */}
        <div className={styles.settingsSection}>
          <h3>Language Preferences</h3>
          <div className={styles.formGroup}>
            <label htmlFor="language">Display Language</label>
            <select
              id="language"
              value={language}
              onChange={(e) => setLanguage(e.target.value)}
              className={styles.select}
            >
              <option value="en">English</option>
              <option value="ur">Urdu</option>
              <option value="es">Spanish</option>
              <option value="fr">French</option>
              <option value="de">German</option>
            </select>
          </div>
        </div>

        {/* Learning Pace */}
        <div className={styles.settingsSection}>
          <h3>Learning Pace</h3>
          <div className={styles.formGroup}>
            <label>
              <input
                type="radio"
                name="pace"
                value="beginner"
                checked={learningPace === 'beginner'}
                onChange={() => setLearningPace('beginner')}
              />
              Beginner
            </label>
            <label>
              <input
                type="radio"
                name="pace"
                value="intermediate"
                checked={learningPace === 'intermediate'}
                onChange={() => setLearningPace('intermediate')}
              />
              Intermediate
            </label>
            <label>
              <input
                type="radio"
                name="pace"
                value="advanced"
                checked={learningPace === 'advanced'}
                onChange={() => setLearningPace('advanced')}
              />
              Advanced
            </label>
          </div>
        </div>

        {/* Preferred Topics */}
        <div className={styles.settingsSection}>
          <h3>Preferred Topics</h3>
          <p>Select topics you're most interested in:</p>
          <div className={styles.topicGrid}>
            {topics.map(topic => (
              <label key={topic} className={styles.topicCheckbox}>
                <input
                  type="checkbox"
                  checked={preferredTopics.includes(topic)}
                  onChange={() => handleTopicToggle(topic)}
                />
                {topic}
              </label>
            ))}
          </div>
        </div>

        {/* Notification Settings */}
        <div className={styles.settingsSection}>
          <h3>Notification Preferences</h3>
          <div className={styles.formGroup}>
            <label>
              <input
                type="checkbox"
                checked={notifications.email}
                onChange={(e) => setNotifications({...notifications, email: e.target.checked})}
              />
              Email Notifications
            </label>
            <label>
              <input
                type="checkbox"
                checked={notifications.push}
                onChange={(e) => setNotifications({...notifications, push: e.target.checked})}
              />
              Push Notifications
            </label>
          </div>
        </div>

        {/* UI Settings */}
        <div className={styles.settingsSection}>
          <h3>UI Preferences</h3>
          
          <div className={styles.formGroup}>
            <label htmlFor="fontSize">Font Size</label>
            <select
              id="fontSize"
              value={uiSettings.fontSize}
              onChange={(e) => setUiSettings({...uiSettings, fontSize: e.target.value})}
              className={styles.select}
            >
              <option value="small">Small</option>
              <option value="medium">Medium</option>
              <option value="large">Large</option>
              <option value="xlarge">Extra Large</option>
            </select>
          </div>
          
          <div className={styles.formGroup}>
            <label htmlFor="theme">Theme</label>
            <select
              id="theme"
              value={uiSettings.theme}
              onChange={(e) => {
                setUiSettings({...uiSettings, theme: e.target.value});
                if (e.target.value !== 'auto') {
                  setColorMode(e.target.value);
                }
              }}
              className={styles.select}
            >
              <option value="auto">Auto (System)</option>
              <option value="light">Light</option>
              <option value="dark">Dark</option>
            </select>
          </div>
          
          <div className={styles.formGroup}>
            <label>
              <input
                type="checkbox"
                checked={uiSettings.showAnimations}
                onChange={(e) => setUiSettings({...uiSettings, showAnimations: e.target.checked})}
              />
              Enable Animations
            </label>
          </div>
        </div>
      </div>

      <div className={styles.actionButtons}>
        <button onClick={handleSave} className={styles.saveButton}>
          Save Preferences
        </button>
        <button 
          onClick={() => {
            // Reset to defaults
            setLanguage('en');
            setLearningPace('intermediate');
            setPreferredTopics([]);
            setNotifications({ email: true, push: false });
            setUiSettings({ fontSize: 'medium', theme: 'auto', showAnimations: true });
          }}
          className={styles.resetButton}
        >
          Reset to Defaults
        </button>
      </div>
    </div>
  );
};

export default PersonalizationSettings;