import pg from 'pg';
import * as dotenv from 'dotenv';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const { Pool } = pg;

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
dotenv.config({ path: join(__dirname, '..', '.env') });

const pool = new Pool({
  connectionString: process.env.NEON_DATABASE_URL,
});

async function verifyTable() {
  const client = await pool.connect();

  try {
    console.log('Checking if personalized_chapters table exists...\n');

    // Check all tables in the database
    const result = await client.query(`
      SELECT table_schema, table_name
      FROM information_schema.tables
      WHERE table_name = 'personalized_chapters'
    `);

    if (result.rows.length === 0) {
      console.log('❌ Table personalized_chapters does NOT exist');
      console.log('\nLet me create it now...\n');

      // Create the table
      await client.query(`
        CREATE TABLE IF NOT EXISTS personalized_chapters (
          id serial PRIMARY KEY NOT NULL,
          user_id text NOT NULL,
          chapter_path text NOT NULL,
          chapter_title text,
          chapter_excerpt text,
          created_at timestamp with time zone DEFAULT now() NOT NULL
        );
      `);

      // Add foreign key constraint
      await client.query(`
        DO $$
        BEGIN
          IF NOT EXISTS (
            SELECT 1 FROM pg_constraint WHERE conname = 'personalized_chapters_user_id_user_id_fk'
          ) THEN
            ALTER TABLE personalized_chapters
            ADD CONSTRAINT personalized_chapters_user_id_user_id_fk
            FOREIGN KEY (user_id) REFERENCES "user"(id) ON DELETE cascade;
          END IF;
        END $$;
      `);

      // Create indexes
      await client.query(`
        CREATE UNIQUE INDEX IF NOT EXISTS idx_personalized_chapters_user_chapter
        ON personalized_chapters USING btree (user_id, chapter_path);
      `);

      await client.query(`
        CREATE INDEX IF NOT EXISTS idx_personalized_chapters_user_id
        ON personalized_chapters USING btree (user_id);
      `);

      await client.query(`
        CREATE INDEX IF NOT EXISTS idx_personalized_chapters_path
        ON personalized_chapters USING btree (chapter_path);
      `);

      console.log('✅ Table personalized_chapters created successfully!');
      console.log('✅ Indexes created successfully!');
    } else {
      console.log('✅ Table personalized_chapters EXISTS in schema:', result.rows[0].table_schema);

      // Verify columns
      const columns = await client.query(`
        SELECT column_name, data_type
        FROM information_schema.columns
        WHERE table_name = 'personalized_chapters'
        ORDER BY ordinal_position
      `);

      console.log('\nColumns:');
      columns.rows.forEach(col => {
        console.log(`  - ${col.column_name}: ${col.data_type}`);
      });

      // Count rows
      const count = await client.query('SELECT COUNT(*) FROM personalized_chapters');
      console.log(`\nRow count: ${count.rows[0].count}`);
    }

  } catch (error) {
    console.error('❌ Error:', error.message);
  } finally {
    client.release();
    await pool.end();
  }
}

verifyTable().catch(console.error);
