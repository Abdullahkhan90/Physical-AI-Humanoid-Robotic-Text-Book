import React, { useState, useRef, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import styles from './Chatbot.module.css';

interface Citation {
  section: string;
  url: string;
  text: string;
}

interface ChatMessage {
  id: string;
  text: string;
  isUser: boolean;
  timestamp: Date;
  sources?: Citation[];
  confidence?: number;
}

const Chatbot = () => {
  const [messages, setMessages] = useState<ChatMessage[]>([
    {
      id: '1',
      text: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you with the content?',
      isUser: false,
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState<string>('');
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  const { colorMode } = useColorMode();

  const toggleChatbot = () => setIsOpen(!isOpen);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    if (isOpen) scrollToBottom();
  }, [messages, isOpen]);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection()?.toString().trim();
      if (selection) setSelectedText(selection);
    };
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      text: inputValue,
      isUser: true,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      let endpoint = '/api/query';
      let body = { question: inputValue };

      if (selectedText) {
        endpoint = '/api/query-selected';
        body = { question: inputValue || selectedText, selected_text: selectedText };
      }

      const isLocal = window.location.hostname === "localhost" || window.location.hostname === "127.0.0.1";
      const baseUrl = isLocal ? "http://localhost:8000" : "";

      const response = await fetch(`${baseUrl}${endpoint}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      });

      if (!response.ok) throw new Error('API error');

      const data = await response.json();

      // Format the response with sources and confidence
      const botResponse: ChatMessage = {
        id: (Date.now() + 1).toString(),
        text: data.answer || 'No answer found in the textbook content.',
        isUser: false,
        timestamp: new Date(),
        sources: data.citations || [],
        confidence: data.confidence || 0.8
      };

      setMessages(prev => [...prev, botResponse]);
      setSelectedText('');
    } catch (err) {
      const errorMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error processing your request. Backend might not be reachable. Run local backend for full functionality.',
        isUser: false,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      if (inputRef.current) {
        inputRef.current.focus();
      }
    }
  };

  // Define styles for the floating elements since they don't fit the CSS module pattern
  const floatingButtonStyle = {
    position: 'fixed' as const,
    bottom: '20px',
    right: '20px',
    width: '60px',
    height: '60px',
    background: 'linear-gradient(135deg, #00d4aa, #00a085)',
    borderRadius: '50%',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    cursor: 'pointer',
    boxShadow: '0 8px 25px rgba(0,212,170,0.4)',
    zIndex: 9998
  };

  const chatWindowStyle = {
    position: 'fixed' as const,
    bottom: '90px',
    right: '20px',
    width: '380px',
    height: '600px',
    borderRadius: '16px',
    boxShadow: '0 15px 40px rgba(0,0,0,0.4)',
    overflow: 'hidden',
    zIndex: 9999,
    display: 'flex' as const,
    flexDirection: 'column' as const,
  };

  return (
    <>
      {/* Floating Icon */}
      <div
        onClick={toggleChatbot}
        style={floatingButtonStyle}
      >
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="white" strokeWidth="2">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      </div>

      {/* Chat Window */}
      {isOpen && (
        <div
          style={{
            ...chatWindowStyle,
            backgroundColor: colorMode === 'dark' ? '#1a1a1a' : '#fff',
            border: colorMode === 'dark' ? '1px solid #333' : '1px solid #ddd'
          }}
        >
          <div className={styles.chatHeader}>
            <h3>Physical AI & Humanoid Robotics Assistant</h3>
            <p>Ask questions about the textbook content</p>
            {selectedText && (
              <div className={styles.selectedTextPreview}>
                Context: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
              </div>
            )}
          </div>

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${message.isUser ? styles.userMessage : styles.botMessage}`}
              >
                <div className={styles.messageContent}>
                  {message.text}
                </div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <h4>Sources:</h4>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx} className={styles.sourceItem}>
                          <span className={styles.sourceTitle}>{source.section}</span>
                          {source.url && (
                            <a href={source.url} target="_blank" rel="noopener noreferrer" className={styles.citationLink}>
                              View
                            </a>
                          )}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
                {message.confidence !== undefined && (
                  <div className={styles.confidence}>
                    Confidence: {(message.confidence * 100).toFixed(0)}%
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about the textbook content..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.chatSubmitButton}
              disabled={isLoading || !inputValue.trim()}
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default Chatbot;