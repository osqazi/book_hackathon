"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
var express_1 = __importDefault(require("express"));
var auth_1 = require("./better-auth/auth");
var express_2 = require("better-auth/express");
var routes_1 = __importDefault(require("../api/personalization/routes"));
var cors_1 = __importDefault(require("cors"));
var app = (0, express_1.default)();
// Enable CORS for all routes
app.use((0, cors_1.default)({
    origin: process.env.NODE_ENV === 'production'
        ? ['https://osqazi.github.io', 'https://book-hackathon-alpha.vercel.app']
        : ['http://localhost:3000', 'http://localhost:3001'],
    credentials: true,
}));
// Parse JSON bodies
app.use(express_1.default.json());
// Use Better-Auth middleware
app.use('/api/auth', (0, express_2.server)(auth_1.auth));
// Use personalization routes
app.use('/api/personalization', routes_1.default);
// Health check endpoint
app.get('/health', function (req, res) {
    res.status(200).json({ status: 'OK', timestamp: new Date().toISOString() });
});
// Error handling middleware
app.use(function (err, req, res, next) {
    console.error('Unhandled error:', err);
    res.status(500).json({
        error: 'INTERNAL_SERVER_ERROR',
        message: 'An unexpected error occurred'
    });
});
// 404 handler
app.use('*', function (req, res) {
    res.status(404).json({
        error: 'NOT_FOUND',
        message: 'Endpoint not found'
    });
});
var PORT = process.env.PORT || 3001;
app.listen(PORT, function () {
    console.log("Server is running on port ".concat(PORT));
});
exports.default = app;
