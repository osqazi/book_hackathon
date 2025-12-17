"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.personalizedChapterSchema = void 0;
var zod_1 = require("zod");
exports.personalizedChapterSchema = zod_1.z.object({
    chapter_path: zod_1.z.string()
        .min(1, 'Chapter path is required')
        .max(500, 'Chapter path too long')
        .regex(/^\/[a-zA-Z0-9/_-]+$/, 'Invalid chapter path format'),
    chapter_title: zod_1.z.string()
        .max(500, 'Chapter title too long')
        .optional()
        .nullable(),
    chapter_excerpt: zod_1.z.string()
        .max(50, 'Chapter excerpt must be 50 characters or less')
        .optional()
        .nullable(),
});
