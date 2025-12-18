// src/components/RecommendedForYou.tsx
import React from 'react';
import Link from '@docusaurus/Link';
import { useAuthContext } from '@site/src/auth/context/AuthProvider';
import { getRecommendedContent, RecommendedContent } from '@site/src/lib/content-recommendations';
import styles from './RecommendedForYou.module.css';

const RecommendedForYou: React.FC = () => {
  const { session } = useAuthContext();

  // Get user's background data
  const softwareBackground = session?.user?.softwareBackground || {};
  const hardwareBackground = session?.user?.hardwareBackground || {};

  // Check if user has completed the questionnaire
  const hasBackground =
    (softwareBackground && Object.keys(softwareBackground).length > 0) ||
    (hardwareBackground && Object.keys(hardwareBackground).length > 0);

  // Get recommendations
  const recommendations = hasBackground
    ? getRecommendedContent(softwareBackground, hardwareBackground)
    : [];

  // If no background data, show prompt to complete profile
  if (!hasBackground) {
    return (
      <div className={styles.emptyState}>
        <h2>Personalized Recommendations</h2>
        <div className={styles.emptyContent}>
          <p>Complete your profile to get personalized content recommendations!</p>
          <p className={styles.emptyHint}>
            Sign up and answer the questionnaire to see chapters tailored to your interests and experience level.
          </p>
        </div>
      </div>
    );
  }

  // If no recommendations (shouldn't happen if hasBackground is true)
  if (recommendations.length === 0) {
    return (
      <div className={styles.emptyState}>
        <h2>Personalized Recommendations</h2>
        <p>No recommendations available at this time.</p>
      </div>
    );
  }

  // Group recommendations by category
  const groupedRecommendations = recommendations.reduce((acc, rec) => {
    if (!acc[rec.category]) {
      acc[rec.category] = [];
    }
    acc[rec.category].push(rec);
    return acc;
  }, {} as Record<string, RecommendedContent[]>);

  const categoryLabels = {
    'getting-started': 'Getting Started',
    'fundamentals': 'Fundamentals',
    'practical': 'Practical Guides',
    'advanced': 'Advanced Topics'
  };

  return (
    <div className={styles.recommendedSection}>
      <h2>📚 Personalized For You Based on Your Interests</h2>
      <p className={styles.subtitle}>
        These chapters are recommended based on your questionnaire responses
      </p>

      {Object.entries(groupedRecommendations).map(([category, items]) => (
        <div key={category} className={styles.categorySection}>
          <h3 className={styles.categoryTitle}>
            {categoryLabels[category as keyof typeof categoryLabels] || category}
          </h3>
          <div className={styles.recommendationsGrid}>
            {items.map((recommendation, index) => (
              <RecommendationCard key={index} recommendation={recommendation} />
            ))}
          </div>
        </div>
      ))}
    </div>
  );
};

interface RecommendationCardProps {
  recommendation: RecommendedContent;
}

const RecommendationCard: React.FC<RecommendationCardProps> = ({ recommendation }) => {
  return (
    <Link to={recommendation.path} className={styles.recommendationCard}>
      <div className={styles.cardContent}>
        <div className={styles.cardHeader}>
          <h4 className={styles.cardTitle}>{recommendation.title}</h4>
          <span className={styles.priorityBadge} data-priority={recommendation.priority}>
            Priority {recommendation.priority}
          </span>
        </div>
        <p className={styles.cardExcerpt}>{recommendation.excerpt}</p>
        <div className={styles.cardFooter}>
          <span className={styles.reason}>💡 {recommendation.reason}</span>
          <span className={styles.readMore}>Read more →</span>
        </div>
      </div>
    </Link>
  );
};

export default RecommendedForYou;
