"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g = Object.create((typeof Iterator === "function" ? Iterator : Object).prototype);
    return g.next = verb(0), g["throw"] = verb(1), g["return"] = verb(2), typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.removePersonalizedChapter = exports.addPersonalizedChapter = exports.getPersonalizedChapters = void 0;
var queries_1 = require("../../lib/db/queries");
var validation_1 = require("./validation");
/**
 * GET /api/personalization/chapters
 * Fetch all personalized chapters for the authenticated user
 */
var getPersonalizedChapters = function (req, res) { return __awaiter(void 0, void 0, void 0, function () {
    var user, chapters, error_1;
    return __generator(this, function (_a) {
        switch (_a.label) {
            case 0:
                _a.trys.push([0, 2, , 3]);
                user = req.auth.user;
                if (!user) {
                    return [2 /*return*/, res.status(401).json({
                            error: 'UNAUTHORIZED',
                            message: 'You must be logged in to access this resource'
                        })];
                }
                return [4 /*yield*/, (0, queries_1.getUserPersonalizedChapters)(user.id)];
            case 1:
                chapters = _a.sent();
                res.status(200).json({
                    chapters: chapters,
                    count: chapters.length
                });
                return [3 /*break*/, 3];
            case 2:
                error_1 = _a.sent();
                console.error('Error fetching personalized chapters:', error_1);
                res.status(500).json({
                    error: 'INTERNAL_SERVER_ERROR',
                    message: 'An unexpected error occurred. Please try again later.'
                });
                return [3 /*break*/, 3];
            case 3: return [2 /*return*/];
        }
    });
}); };
exports.getPersonalizedChapters = getPersonalizedChapters;
/**
 * POST /api/personalization/chapters
 * Add a chapter to user's personalizations
 */
var addPersonalizedChapter = function (req, res) { return __awaiter(void 0, void 0, void 0, function () {
    var user, validationResult, _a, chapter_path_1, chapter_title, chapter_excerpt, result, existingChapter, chapters, chapter, error_2;
    return __generator(this, function (_b) {
        switch (_b.label) {
            case 0:
                _b.trys.push([0, 7, , 8]);
                user = req.auth.user;
                if (!user) {
                    return [2 /*return*/, res.status(401).json({
                            error: 'UNAUTHORIZED',
                            message: 'You must be logged in to access this resource'
                        })];
                }
                validationResult = validation_1.personalizedChapterSchema.safeParse(req.body);
                if (!validationResult.success) {
                    return [2 /*return*/, res.status(422).json({
                            error: 'VALIDATION_ERROR',
                            message: 'Request validation failed',
                            validation_errors: validationResult.error.errors.map(function (err) { return ({
                                field: err.path.join('.'),
                                message: err.message
                            }); })
                        })];
                }
                _a = validationResult.data, chapter_path_1 = _a.chapter_path, chapter_title = _a.chapter_title, chapter_excerpt = _a.chapter_excerpt;
                return [4 /*yield*/, (0, exports.addPersonalizedChapter)(user.id, chapter_path_1, chapter_title, chapter_excerpt)];
            case 1:
                result = _b.sent();
                if (!(result.length > 0)) return [3 /*break*/, 2];
                // Chapter was added
                res.status(201).json(result[0]);
                return [3 /*break*/, 6];
            case 2: return [4 /*yield*/, (0, queries_1.isChapterPersonalized)(user.id, chapter_path_1)];
            case 3:
                existingChapter = _b.sent();
                if (!existingChapter) return [3 /*break*/, 5];
                return [4 /*yield*/, (0, queries_1.getUserPersonalizedChapters)(user.id)];
            case 4:
                chapters = _b.sent();
                chapter = chapters.find(function (c) { return c.chapterPath === chapter_path_1; });
                if (chapter) {
                    res.status(200).json(chapter);
                }
                else {
                    res.status(500).json({
                        error: 'INTERNAL_SERVER_ERROR',
                        message: 'Chapter was not found after insertion'
                    });
                }
                return [3 /*break*/, 6];
            case 5:
                res.status(500).json({
                    error: 'INTERNAL_SERVER_ERROR',
                    message: 'Chapter was not added to the database'
                });
                _b.label = 6;
            case 6: return [3 /*break*/, 8];
            case 7:
                error_2 = _b.sent();
                console.error('Error adding personalized chapter:', error_2);
                res.status(500).json({
                    error: 'INTERNAL_SERVER_ERROR',
                    message: 'An unexpected error occurred. Please try again later.'
                });
                return [3 /*break*/, 8];
            case 8: return [2 /*return*/];
        }
    });
}); };
exports.addPersonalizedChapter = addPersonalizedChapter;
/**
 * DELETE /api/personalization/chapters/:chapterPath
 * Remove a chapter from user's personalizations
 */
var removePersonalizedChapter = function (req, res) { return __awaiter(void 0, void 0, void 0, function () {
    var user, chapterPath, decodedChapterPath, result, error_3;
    return __generator(this, function (_a) {
        switch (_a.label) {
            case 0:
                _a.trys.push([0, 2, , 3]);
                user = req.auth.user;
                if (!user) {
                    return [2 /*return*/, res.status(401).json({
                            error: 'UNAUTHORIZED',
                            message: 'You must be logged in to access this resource'
                        })];
                }
                chapterPath = req.params.chapterPath;
                // Validate chapter path format
                if (!chapterPath || !/^\/[a-zA-Z0-9/_-]+$/.test(decodeURIComponent(chapterPath))) {
                    return [2 /*return*/, res.status(400).json({
                            error: 'BAD_REQUEST',
                            message: 'Invalid chapter path format'
                        })];
                }
                decodedChapterPath = decodeURIComponent(chapterPath);
                return [4 /*yield*/, (0, exports.removePersonalizedChapter)(user.id, decodedChapterPath)];
            case 1:
                result = _a.sent();
                // Idempotent operation - return 204 regardless of whether the chapter existed
                res.status(204).send();
                return [3 /*break*/, 3];
            case 2:
                error_3 = _a.sent();
                console.error('Error removing personalized chapter:', error_3);
                res.status(500).json({
                    error: 'INTERNAL_SERVER_ERROR',
                    message: 'An unexpected error occurred. Please try again later.'
                });
                return [3 /*break*/, 3];
            case 3: return [2 /*return*/];
        }
    });
}); };
exports.removePersonalizedChapter = removePersonalizedChapter;
