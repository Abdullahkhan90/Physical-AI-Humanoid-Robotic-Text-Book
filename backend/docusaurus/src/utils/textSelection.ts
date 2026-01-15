/**
 * Text selection utility for the Docusaurus-based textbook
 * Provides functionality to get selected text and show floating action button
 */

// Function to get currently selected text
export const getSelectedText = () => {
  return window.getSelection ? window.getSelection().toString() : '';
};

// Function to get the position of the selected text
export const getSelectionPosition = () => {
  const selection = window.getSelection();
  if (!selection || selection.rangeCount === 0) return null;
  
  const range = selection.getRangeAt(0);
  const rect = range.getBoundingClientRect();
  
  return {
    x: rect.left + window.scrollX,
    y: rect.top + window.scrollY,
    width: rect.width,
    height: rect.height
  };
};

// Function to determine if text is selected
export const isTextSelected = () => {
  return getSelectedText().length > 0;
};

// Function to create a floating button for asking about selected text
export const createFloatingButton = (onAskClick) => {
  // Remove any existing floating buttons
  removeFloatingButton();
  
  // Create the button element
  const button = document.createElement('div');
  button.id = 'ask-about-text-button';
  button.style.position = 'absolute';
  button.style.zIndex = '10000';
  button.style.backgroundColor = '#4f46e5';
  button.style.color = 'white';
  button.style.border = 'none';
  button.style.borderRadius = '4px';
  button.style.padding = '8px 12px';
  button.style.fontSize = '14px';
  button.style.cursor = 'pointer';
  button.style.boxShadow = '0 4px 6px rgba(0, 0, 0, 0.1)';
  button.style.display = 'flex';
  button.style.alignItems = 'center';
  button.style.gap = '4px';
  
  button.innerHTML = '💬 Ask about this';
  
  // Position the button near the selected text
  const position = getSelectionPosition();
  if (position) {
    button.style.left = `${position.x}px`;
    button.style.top = `${position.y - 40}px`; // Position above the selection
  }
  
  // Add click event
  button.addEventListener('click', (e) => {
    e.preventDefault();
    e.stopPropagation();
    const selectedText = getSelectedText();
    onAskClick(selectedText);
    removeFloatingButton();
  });
  
  // Add to the document
  document.body.appendChild(button);
  
  return button;
};

// Function to remove the floating button
export const removeFloatingButton = () => {
  const existingButton = document.getElementById('ask-about-text-button');
  if (existingButton) {
    existingButton.remove();
  }
};

// Initialize text selection functionality
export const initializeTextSelection = (onAskClick) => {
  let mouseUpTimer;
  
  const handleMouseUp = () => {
    // Clear any existing timer
    if (mouseUpTimer) {
      clearTimeout(mouseUpTimer);
    }
    
    // Use a timer to ensure selection has been completed
    mouseUpTimer = setTimeout(() => {
      if (isTextSelected()) {
        createFloatingButton(onAskClick);
      } else {
        removeFloatingButton();
      }
    }, 100);
  };
  
  // Add event listeners
  document.addEventListener('mouseup', handleMouseUp);
  
  // Remove event listeners when needed
  return () => {
    document.removeEventListener('mouseup', handleMouseUp);
    if (mouseUpTimer) {
      clearTimeout(mouseUpTimer);
    }
    removeFloatingButton();
  };
};