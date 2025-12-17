import { z } from 'zod';

export const personalizedChapterSchema = z.object({
  chapter_path: z.string()
    .min(1, 'Chapter path is required')
    .max(500, 'Chapter path too long')
    .regex(/^\/[a-zA-Z0-9/_-]+$/, 'Invalid chapter path format'),

  chapter_title: z.string()
    .max(500, 'Chapter title too long')
    .optional()
    .nullable(),

  chapter_excerpt: z.string()
    .max(50, 'Chapter excerpt must be 50 characters or less')
    .optional()
    .nullable(),
});

export type PersonalizedChapterInput = z.infer<typeof personalizedChapterSchema>;