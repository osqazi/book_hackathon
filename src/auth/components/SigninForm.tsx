// src/auth/components/SigninForm.tsx
import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import { SignInData } from '../../models/user';

interface SigninFormProps {
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

const SigninForm: React.FC<SigninFormProps> = ({ onSuccess, onError }) => {
  const { signIn, isLoading } = useAuth();
  const [formData, setFormData] = useState<SignInData>({
    email: '',
    password: '',
  });
  const [error, setError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    try {
      const result = await signIn({
        email: formData.email,
        password: formData.password,
      });

      // Check if error is truthy (not just if the property exists)
      if (result.error) {
        const errorMsg = typeof result.error === 'string'
          ? result.error
          : result.error?.message || result.error?.statusText || 'Sign in failed';
        setError(errorMsg);
        onError?.(errorMsg);
      } else {
        // Success - error is null
        onSuccess?.();
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Sign in failed';
      setError(errorMessage);
      onError?.(errorMessage);
    }
  };

  return (
    <div className="signin-form">
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
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
            disabled={isLoading}
          />
        </div>

        {error && <div className="error-message">{error}</div>}

        <button type="submit" disabled={isLoading}>
          {isLoading ? 'Signing in...' : 'Sign In'}
        </button>
      </form>
    </div>
  );
};

export default SigninForm;
