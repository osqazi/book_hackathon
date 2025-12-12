// src/auth/components/SignupForm.tsx
import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import { RegistrationData } from '../../models/user';
import BackgroundQuestions from './BackgroundQuestions';

interface SignupFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

const SignupForm: React.FC<SignupFormProps> = ({ onSuccess, onError }) => {
  const { signUp, isLoading } = useAuth();
  const [formData, setFormData] = useState<Omit<RegistrationData, 'softwareBackground' | 'hardwareBackground'> & {
    softwareBackground: { [key: string]: any };
    hardwareBackground: { [key: string]: any };
  }>({
    email: '',
    password: '',
    softwareBackground: {},
    hardwareBackground: {},
  });
  const [error, setError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleBackgroundChange = (category: 'software' | 'hardware', responses: { [key: string]: any }) => {
    setFormData(prev => ({
      ...prev,
      [`${category}Background`]: responses
    }));
  };

  const validateForm = (): boolean => {
    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(formData.email)) {
      setError('Please enter a valid email address');
      return false;
    }

    // Password validation
    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters long');
      return false;
    }

    return true;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    // Validate form before submitting
    if (!validateForm()) {
      return;
    }

    try {
      const result = await signUp({
        email: formData.email,
        password: formData.password,
        softwareBackground: formData.softwareBackground,
        hardwareBackground: formData.hardwareBackground,
      });

      // Check if error is truthy (not just if the property exists)
      if (result.error) {
        // Extract error message from error object
        const errorMsg = typeof result.error === 'string'
          ? result.error
          : result.error?.message || result.error?.statusText || 'Sign up failed';
        setError(errorMsg);
        onError?.(errorMsg);
      } else {
        // Success - error is null
        onSuccess?.();
      }
    } catch (err) {
      // Handle thrown errors
      let errorMessage = 'Sign up failed';
      if (err instanceof Error) {
        errorMessage = err.message;
      } else if (typeof err === 'object' && err !== null) {
        // Handle error objects with status/statusText
        errorMessage = (err as any).message || (err as any).statusText || JSON.stringify(err);
      } else if (typeof err === 'string') {
        errorMessage = err;
      }
      setError(errorMessage);
      onError?.(errorMessage);
    }
  };

  return (
    <div className="signup-form">
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
            disabled={isLoading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password (minimum 8 characters):</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
            minLength={8}
            disabled={isLoading}
          />
        </div>

        <div className="background-questions-section">
          <h3>Tell us about your background:</h3>
          <BackgroundQuestions
            onResponsesChange={handleBackgroundChange}
          />
        </div>

        {error && <div className="error-message">{error}</div>}

        <button type="submit" disabled={isLoading}>
          {isLoading ? 'Signing up...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;