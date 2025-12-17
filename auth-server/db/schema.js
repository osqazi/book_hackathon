// auth-server/db/schema.js
import { pgTable, serial, text, timestamp, uniqueIndex, index } from 'drizzle-orm/pg-core';

// Reference to existing Better-Auth user table
export const user = pgTable('user', {
  id: text('id').primaryKey(),
  email: text('email').notNull(),
  name: text('name'),
});

// Personalized chapters table
export const personalizedChapter = pgTable(
  'personalized_chapters',
  {
    id: serial('id').primaryKey(),
    userId: text('user_id')
      .notNull()
      .references(() => user.id, { onDelete: 'cascade' }),
    chapterPath: text('chapter_path').notNull(),
    chapterTitle: text('chapter_title'),
    chapterExcerpt: text('chapter_excerpt'),
    createdAt: timestamp('created_at', { withTimezone: true }).defaultNow().notNull(),
  },
  (table) => ({
    userChapterIdx: uniqueIndex('idx_personalized_chapters_user_chapter').on(
      table.userId,
      table.chapterPath
    ),
    userIdIdx: index('idx_personalized_chapters_user_id').on(table.userId),
    chapterPathIdx: index('idx_personalized_chapters_path').on(table.chapterPath),
  })
);
