"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.db = void 0;
var node_postgres_1 = require("drizzle-orm/node-postgres");
var pg_1 = require("pg");
// Create PostgreSQL connection pool
var pool = new pg_1.Pool({
    connectionString: process.env.DATABASE_URL,
});
// Create Drizzle ORM client
exports.db = (0, node_postgres_1.drizzle)(pool);
