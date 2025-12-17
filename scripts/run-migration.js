import pg from 'pg';
import * as dotenv from 'dotenv';
import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const { Pool } = pg;

// Load environment variables
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
dotenv.config({ path: join(__dirname, '..', '.env') });

const pool = new Pool({
  connectionString: process.env.NEON_DATABASE_URL,
});

async function runMigration() {
  const client = await pool.connect();

  try {
    console.log('Connecting to database...');

    // Read the migration SQL file
    const migrationSQL = readFileSync(
      join(__dirname, '..', 'drizzle', 'migrations', '0000_previous_garia.sql'),
      'utf-8'
    );

    console.log('Running migration...');

    // Execute the migration
    await client.query(migrationSQL);

    console.log('✅ Migration completed successfully!');
    console.log('Created table: personalized_chapters');

  } catch (error) {
    if (error.code === '42P07') {
      console.log('ℹ️  Table already exists, skipping migration.');
    } else {
      console.error('❌ Migration failed:', error.message);
      throw error;
    }
  } finally {
    client.release();
    await pool.end();
  }
}

runMigration().catch(console.error);
