import { drizzle } from 'drizzle-orm/node-postgres';
import { Pool } from 'pg';

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

// Create Drizzle ORM client
export const db = drizzle(pool);