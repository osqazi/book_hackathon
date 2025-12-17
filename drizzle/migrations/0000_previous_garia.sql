CREATE TABLE "personalized_chapters" (
	"id" serial PRIMARY KEY NOT NULL,
	"user_id" text NOT NULL,
	"chapter_path" text NOT NULL,
	"chapter_title" text,
	"chapter_excerpt" text,
	"created_at" timestamp with time zone DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE "user" (
	"id" text PRIMARY KEY NOT NULL,
	"email" text NOT NULL,
	"name" text
);
--> statement-breakpoint
ALTER TABLE "personalized_chapters" ADD CONSTRAINT "personalized_chapters_user_id_user_id_fk" FOREIGN KEY ("user_id") REFERENCES "public"."user"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
CREATE UNIQUE INDEX "idx_personalized_chapters_user_chapter" ON "personalized_chapters" USING btree ("user_id","chapter_path");--> statement-breakpoint
CREATE INDEX "idx_personalized_chapters_user_id" ON "personalized_chapters" USING btree ("user_id");--> statement-breakpoint
CREATE INDEX "idx_personalized_chapters_path" ON "personalized_chapters" USING btree ("chapter_path");