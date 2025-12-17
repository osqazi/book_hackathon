"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
var express_1 = __importDefault(require("express"));
var handlers_1 = require("./handlers");
var express_2 = require("better-auth/integrations/express");
var router = express_1.default.Router();
// GET /api/personalization/chapters - Fetch all personalized chapters for user
router.get('/chapters', (0, express_2.withAuth)(), handlers_1.getPersonalizedChapters);
// POST /api/personalization/chapters - Add a chapter to personalizations
router.post('/chapters', (0, express_2.withAuth)(), handlers_1.addPersonalizedChapter);
// DELETE /api/personalization/chapters/:chapterPath - Remove a chapter from personalizations
router.delete('/chapters/:chapterPath', (0, express_2.withAuth)(), handlers_1.removePersonalizedChapter);
exports.default = router;
