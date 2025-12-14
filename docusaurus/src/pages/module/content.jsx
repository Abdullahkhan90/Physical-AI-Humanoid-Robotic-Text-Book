import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import ContentDisplay from '../../components/ContentDisplay';
import { LearningContent } from '../../models/entities';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useParams } from '@docusaurus/router';

// Mock data for content - in a real implementation this would come from an API
const mockContent = {
  "content-1-1": {
    id: "content-1-1",
    title: "Introduction to ROS 2",
    content: "# Introduction to ROS 2\n\nROS 2 (Robot Operating System 2) is a flexible framework for writing robot applications. It is the next generation of the Robot Operating System...",
    moduleID: "module-1-ros2",
    contentType: "theory",
    wordCount: 1200,
    estimatedReadingTime: 5,
    requiredCitations: [
      {
        id: "cit-001",
        title: "A survey of robot software frameworks",
        authors: ["Smith, J.", "Johnson, A."],
        source: "Journal of Robotics",
        year: 2022,
        doi: "10.1155/2022/1234567",
        url: "https://www.hindawi.com/journals/jr/2022/1234567/",
        citationType: "journalArticle",
        apaFormatted: "Smith, J., & Johnson, A. (2022). A survey of robot software frameworks. Journal of Robotics. https://doi.org/10.1155/2022/1234567"
      }
    ]
  },
  "content-1-2": {
    id: "content-1-2",
    title: "ROS 2 Nodes, Topics, and Services",
    content: "# ROS 2 Nodes, Topics, and Services\n\nIn ROS 2, nodes communicate through a publish-subscribe model using topics, services, and actions...",
    moduleID: "module-1-ros2",
    contentType: "theory",
    wordCount: 1500,
    estimatedReadingTime: 6,
    requiredCitations: []
  },
  "content-2-1": {
    id: "content-2-1",
    title: "Introduction to Gazebo Simulation",
    content: "# Introduction to Gazebo Simulation\n\nGazebo is a robotics simulator that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces...",
    moduleID: "module-2-digital-twin",
    contentType: "theory",
    wordCount: 1300,
    estimatedReadingTime: 5,
    requiredCitations: []
  }
};

// Module content mapping - in a real implementation this would be dynamic
const moduleContentMap = {
  "ros2": ["content-1-1", "content-1-2"],
  "gazebo": ["content-2-1"],
  "isaac": []
};

export default function ModuleContentPage() {
  const { siteConfig } = useDocusaurusContext();

  const [content, setContent] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [moduleId, setModuleId] = useState(null);

  useEffect(() => {
    // Access window.location only in the browser (client-side)
    if (typeof window !== 'undefined') {
      // Extract module id from query parameters (e.g., ?module=ros2)
      const urlParams = new URLSearchParams(window.location.search);
      const moduleParam = urlParams.get('module');

      if (moduleParam) {
        setModuleId(moduleParam);
      } else {
        // Default to ros2 if no module is specified
        setModuleId('ros2');
      }
    } else {
      // On the server side, set a default module ID
      setModuleId('ros2');
    }
  }, []);

  useEffect(() => {
    if (moduleId) {
      try {
        // In a real implementation, this would fetch data from an API based on moduleId
        // For now, we're using mock data
        const contentIds = moduleContentMap[moduleId] || [];
        if (contentIds.length > 0) {
          // For this example, we'll display the first piece of content
          const contentId = contentIds[0];
          const selectedContent = mockContent[contentId];

          if (selectedContent) {
            setContent(selectedContent);
          } else {
            setError(`Content not found for module: ${moduleId}`);
          }
        } else {
          setError(`No content found for module: ${moduleId}`);
        }
      } catch (err) {
        setError(`Error loading content for module: ${moduleId}`);
        console.error(err);
      } finally {
        setLoading(false);
      }
    }
  }, [moduleId]);

  if (loading) {
    return (
      <Layout title={`Loading... | ${siteConfig.title}`} description="Loading module content">
        <div className="container margin-vert--lg">
          <div className="text--center">
            <p>Loading content...</p>
          </div>
        </div>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title={`Error | ${siteConfig.title}`} description="Error loading module content">
        <div className="container margin-vert--lg">
          <div className="text--center">
            <p>{error}</p>
            <a href="/modules">Go back to modules</a>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout 
      title={`${content?.title || 'Module Content'} | ${siteConfig.title}`} 
      description={content?.title ? `Content for module: ${content.title}` : "Module content"}>
      <main>
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--12">
              {content ? (
                <ContentDisplay content={content} />
              ) : (
                <div className="text--center">
                  <p>No content available for this module.</p>
                </div>
              )}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}