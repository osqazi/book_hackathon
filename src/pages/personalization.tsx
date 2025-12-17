import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { usePersonalizedChapters } from '@site/src/hooks/usePersonalization';
import PersonalizationCard from '@site/src/components/PersonalizationCard';
import PersonalizationEmptyState from '@site/src/components/PersonalizationEmptyState';
import { useAuthContext } from '@site/src/auth/context/AuthProvider';

const PersonalizationPage: React.FC = () => {
  const { session, isLoading: sessionLoading } = useAuthContext();
  const { data: chapters, isLoading, isError } = usePersonalizedChapters();

  // Check if user is authenticated
  if (!sessionLoading && !session) {
    return (
      <Layout title="Personalized Chapters" description="Please log in to view personalized chapters">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="text--center padding-vert--lg">
                <h1>Access Denied</h1>
                <p>You need to be logged in to view your personalized chapters.</p>
                <Link to="/auth/signin" className="button button--primary">
                  Sign In
                </Link>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (sessionLoading) {
    return (
      <Layout title="Loading..." description="Loading your personalized chapters">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="text--center padding-vert--lg">
                <p>Loading...</p>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (isLoading) {
    return (
      <Layout title="Personalized Chapters" description="Your bookmarked documentation chapters">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--10 col--offset-1">
              <h1 className="text--center">Personalized Chapters</h1>
              <div className="text--center padding-vert--lg">
                <p>Loading your personalized chapters...</p>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (isError) {
    return (
      <Layout title="Error" description="Error loading personalized chapters">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="text--center padding-vert--lg">
                <h1>Error Loading Chapters</h1>
                <p>There was an error loading your personalized chapters. Please try again later.</p>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Personalized Chapters" description="Your bookmarked documentation chapters">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <h1 className="text--center">Personalized by You</h1>

            {chapters && chapters.length > 0 ? (
              <div className="personalization-grid">
                {chapters.map((chapter) => (
                  <PersonalizationCard
                    key={chapter.id}
                    id={chapter.id}
                    chapterPath={chapter.chapter_path}
                    chapterTitle={chapter.chapter_title || 'Untitled Chapter'}
                    chapterExcerpt={chapter.chapter_excerpt}
                    createdAt={chapter.created_at}
                  />
                ))}
              </div>
            ) : (
              <PersonalizationEmptyState />
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default PersonalizationPage;