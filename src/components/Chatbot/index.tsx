
import React, { useState, useRef, useEffect } from 'react';
import styles from './Chatbot.module.css';

type Message = {
  id: number;
  text: string;
  sender: 'user' | 'bot';
};

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: 1,
      text: "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book. How can I help you today?",
      sender: 'bot',
    },
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChatbot = () => setIsOpen(!isOpen);
  const closeChatbot = () => setIsOpen(false);

  const handleSend = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userText = inputValue;

    setMessages(prev => [
      ...prev,
      { id: Date.now(), text: userText, sender: 'user' },
    ]);

    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userText, // ✅ ONLY THIS (backend expects this)
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      let botText = data.answer || 'No answer found.';

      if (data.sources && data.sources.length > 0) {
        botText += '\n\nSources:\n';
        data.sources.forEach((s: any, i: number) => {
          botText += `${i + 1}. ${s.source || 'Book Reference'}\n`;
        });
      }

      setMessages(prev => [
        ...prev,
        { id: Date.now() + 1, text: botText, sender: 'bot' },
      ]);
    } catch (error) {
      setMessages(prev => [
        ...prev,
        {
          id: Date.now() + 1,
          text: 'Sorry, I encountered an error while processing your request.',
          sender: 'bot',
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className={`${styles.chatbotButton} ${isOpen ? styles.open : ''}`}
        onClick={toggleChatbot}
        aria-label="Open chatbot"
      >
        💬
      </button>

      {/* Chat Window */}
      <div className={`${styles.chatbotModal} ${isOpen ? styles.open : ''}`}>
        <div className={styles.chatbotHeader}>
          <h3>AI Book Assistant</h3>
          <button onClick={closeChatbot}>×</button>
        </div>

        <div className={styles.chatbotMessages}>
          {messages.map(msg => (
            <div
              key={msg.id}
              className={`${styles.message} ${styles[`${msg.sender}Message`]}`}
            >
              {msg.text.split('\n').map((line, i) => (
                <React.Fragment key={i}>
                  {line}
                  <br />
                </React.Fragment>
              ))}
            </div>
          ))}

          {isLoading && (
            <div className={`${styles.message} ${styles.botMessage}`}>
              <em>Thinking...</em>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        <div className={styles.chatbotInputArea}>
          <input
            type="text"
            value={inputValue}
            onChange={e => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask from the book..."
            disabled={isLoading}
          />
          <button onClick={handleSend} disabled={isLoading || !inputValue.trim()}>
            ➤
          </button>
        </div>
      </div>
    </>
  );
};

export default Chatbot;
