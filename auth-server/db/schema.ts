import { pgTable, serial, text, timestamp, uniqueIndex, index } from 'drizzle-orm/pg-core';
import { relations } from 'drizzle-orm';

// Reference to existing Better-Auth user table (handled by Better-Auth's PostgreSQL adapter)
export const user = pgTable('user', {
  id: text('id').primaryKey(),
  email: text('email').notNull(),
  name: text('name'),
  // ... other Better-Auth fields (managed by Better-Auth)
});

// New table for personalized chapters
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
    // Unique constraint: one user can't personalize same chapter twice
    userChapterIdx: uniqueIndex('idx_personalized_chapters_user_chapter').on(
      table.userId,
      table.chapterPath
    ),
    // Index for fetching all chapters for a user
    userIdIdx: index('idx_personalized_chapters_user_id').on(table.userId),
    // Index for checking if specific chapter is personalized
    chapterPathIdx: index('idx_personalized_chapters_path').on(table.chapterPath),
  })
);

// Relations (for Drizzle's relational query API)
export const userRelations = relations(user, ({ many }) => ({
  personalizedChapters: many(personalizedChapter),
}));

export const personalizedChapterRelations = relations(personalizedChapter, ({ one }) => ({
  user: one(user, {
    fields: [personalizedChapter.userId],
    references: [user.id],
  }),
}));

// TypeScript types
export type PersonalizedChapter = typeof personalizedChapter.$inferSelect;
export type NewPersonalizedChapter = typeof personalizedChapter.$inferInsert;