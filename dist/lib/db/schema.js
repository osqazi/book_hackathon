"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.personalizedChapterRelations = exports.userRelations = exports.personalizedChapter = exports.user = void 0;
var pg_core_1 = require("drizzle-orm/pg-core");
var drizzle_orm_1 = require("drizzle-orm");
// Reference to existing Better-Auth user table (handled by Better-Auth's PostgreSQL adapter)
exports.user = (0, pg_core_1.pgTable)('user', {
    id: (0, pg_core_1.text)('id').primaryKey(),
    email: (0, pg_core_1.text)('email').notNull(),
    name: (0, pg_core_1.text)('name'),
    // ... other Better-Auth fields (managed by Better-Auth)
});
// New table for personalized chapters
exports.personalizedChapter = (0, pg_core_1.pgTable)('personalized_chapters', {
    id: (0, pg_core_1.serial)('id').primaryKey(),
    userId: (0, pg_core_1.text)('user_id')
        .notNull()
        .references(function () { return exports.user.id; }, { onDelete: 'cascade' }),
    chapterPath: (0, pg_core_1.text)('chapter_path').notNull(),
    chapterTitle: (0, pg_core_1.text)('chapter_title'),
    chapterExcerpt: (0, pg_core_1.text)('chapter_excerpt'),
    createdAt: (0, pg_core_1.timestamp)('created_at', { withTimezone: true }).defaultNow().notNull(),
}, function (table) { return ({
    // Unique constraint: one user can't personalize same chapter twice
    userChapterIdx: (0, pg_core_1.uniqueIndex)('idx_personalized_chapters_user_chapter').on(table.userId, table.chapterPath),
    // Index for fetching all chapters for a user
    userIdIdx: (0, pg_core_1.index)('idx_personalized_chapters_user_id').on(table.userId),
    // Index for checking if specific chapter is personalized
    chapterPathIdx: (0, pg_core_1.index)('idx_personalized_chapters_path').on(table.chapterPath),
}); });
// Relations (for Drizzle's relational query API)
exports.userRelations = (0, drizzle_orm_1.relations)(exports.user, function (_a) {
    var many = _a.many;
    return ({
        personalizedChapters: many(exports.personalizedChapter),
    });
});
exports.personalizedChapterRelations = (0, drizzle_orm_1.relations)(exports.personalizedChapter, function (_a) {
    var one = _a.one;
    return ({
        user: one(exports.user, {
            fields: [exports.personalizedChapter.userId],
            references: [exports.user.id],
        }),
    });
});
