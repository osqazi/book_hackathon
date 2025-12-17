"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.auth = void 0;
var better_auth_1 = require("better-auth");
var drizzle_1 = require("better-auth/adapters/drizzle");
var better_sqlite3_1 = __importDefault(require("better-sqlite3"));
// Create SQLite database instance
var sqlite = new better_sqlite3_1.default("sqlite.db");
exports.auth = (0, better_auth_1.betterAuth)({
    secret: process.env.BETTER_AUTH_SECRET || "",
    database: (0, drizzle_1.drizzleAdapter)(sqlite, {
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
