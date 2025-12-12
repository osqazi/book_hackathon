import { drizzle } from "drizzle-orm/better-sqlite3";
import Database from "better-sqlite3";
import * as schema from "./schema";

// Create the SQLite database instance
const sqlite = new Database("sqlite.db");

// Create the Drizzle ORM instance
export const db = drizzle(sqlite, { schema });

// Export the database instance for direct use if needed
export default sqlite;