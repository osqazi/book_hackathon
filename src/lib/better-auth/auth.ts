import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import Database from "better-sqlite3";

// Create SQLite database instance
const sqlite = new Database("sqlite.db");

export const auth = betterAuth({
  database: drizzleAdapter(sqlite, {
    provider: "sqlite",
  }),
  // Custom fields for background profiling
  user: {
    fields: {
      softwareBackground: "json",
      hardwareBackground: "json",
      backgroundComplete: "boolean",
    },
  },
  // Session configuration (7 days as specified)
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days in seconds
  },
  // Other configurations
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // For now, can be enabled later
    password: {
      // Minimum 8 characters as specified
      minLength: 8,
    },
  },
  // Add custom endpoints for updating user profile
  plugins: [
    // Add custom endpoints plugin if needed
  ],
});