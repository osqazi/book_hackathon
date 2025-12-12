// src/auth/hooks/useAuth.ts
import { useAuthContext } from '../context/AuthProvider';

export const useAuth = () => {
  return useAuthContext();
};
