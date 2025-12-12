// src/auth/components/BackgroundQuestions.tsx
import React, { useState, useEffect } from 'react';
import { BackgroundQuestion } from '../../models/user';

interface BackgroundQuestionsProps {
  onResponsesChange: (category: 'software' | 'hardware', responses: { [key: string]: any }) => void;
}

const BackgroundQuestions: React.FC<BackgroundQuestionsProps> = ({ onResponsesChange }) => {
  // We'll fetch questions from a reference data file or API
  // For now, we'll use a static set of questions
  const [questions, setQuestions] = useState<BackgroundQuestion[]>([]);
  const [responses, setResponses] = useState<{ [questionId: string]: string | string[] }>({});

  // Load background questions reference data
  useEffect(() => {
    import('../../lib/background-questions').then((module) => {
      setQuestions(module.backgroundQuestions);
    });
  }, []);

  // Handle changes to question responses
  const handleResponseChange = (questionId: string, value: string | string[]) => {
    const updatedResponses = { ...responses, [questionId]: value };
    setResponses(updatedResponses);

    // Group responses by category and notify parent
    const softwareResponses: { [key: string]: any } = {};
    const hardwareResponses: { [key: string]: any } = {};

    questions.forEach(question => {
      const response = updatedResponses[question.id];
      if (response !== undefined) {
        if (question.category === 'software') {
          softwareResponses[question.id] = response;
        } else if (question.category === 'hardware') {
          hardwareResponses[question.id] = response;
        }
      }
    });

    onResponsesChange('software', softwareResponses);
    onResponsesChange('hardware', hardwareResponses);
  };

  // Handle multiple choice selection
  const handleMultipleChoice = (questionId: string, option: string, selected: boolean) => {
    const currentResponse = responses[questionId] as string[] | undefined;
    let newResponse: string[];

    if (Array.isArray(currentResponse)) {
      newResponse = selected
        ? [...currentResponse, option]
        : currentResponse.filter(item => item !== option);
    } else {
      newResponse = selected ? [option] : [];
    }

    handleResponseChange(questionId, newResponse);
  };

  return (
    <div className="background-questions">
      {questions.map((question) => (
        <div key={question.id} className="question-group">
          <label>{question.questionText}{question.required && ' *'}</label>
          <div className="options">
            {question.options.map((option) => (
              <div key={option} className="option">
                <input
                  type="checkbox"
                  id={`${question.id}-${option}`}
                  onChange={(e) => handleMultipleChoice(question.id, option, e.target.checked)}
                />
                <label htmlFor={`${question.id}-${option}`}>{option}</label>
              </div>
            ))}
          </div>
        </div>
      ))}
    </div>
  );
};

export default BackgroundQuestions;