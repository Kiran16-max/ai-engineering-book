---
id: 14
title: "Add Navigation Buttons to Homepage Features"
stage: green
date: 2025-12-13
surface: agent
model: gemini
feature: general
branch: main
user: user
command: "Update the featured section on my book website. Currently, it has 3 images with titles. I want the following behavior: 1. Under each image, add a button. 2. When the button under Image 1 is clicked, it navigates to Module 1 page. 3. When the button under Image 2 is clicked, it navigates to Module 2 page. 4. When the button under Image 3 is clicked, it navigates to Module 3 page. Ensure the buttons match the website's styling and the layout remains responsive. Do not change the images or titles, only add the buttons with proper navigation to the corresponding module pages."
labels:
  - "ui"
  - "react"
  - "docusaurus"
  - "feature"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "src/components/HomepageFeatures/index.tsx"
  - "src/components/HomepageFeatures/styles.module.css"
tests:
  - 
---

# Prompt

Update the featured section on my book website. Currently, it has 3 images with titles. I want the following behavior:

1. Under each image, add a button.
2. When the button under Image 1 is clicked, it navigates to Module 1 page.
3. When the button under Image 2 is clicked, it navigates to Module 2 page.
4. When the button under Image 3 is clicked, it navigates to Module 3 page.

Ensure the buttons match the website's styling and the layout remains responsive. Do not change the images or titles, only add the buttons with proper navigation to the corresponding module pages.

# Response

I have added buttons to the featured section on your website. The buttons navigate to the corresponding module pages as you requested. I also fixed a syntax error and a broken link that were preventing the site from building.
