\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{comp450}[2010/02/05]
\LoadClass[11pt,letterpaper]{article}

\RequirePackage{graphicx, wrapfig, xspace, mdwlist, amsmath, amssymb, mathptmx, helvet, color, microtype, titlesec, booktabs, comment, listings}
\RequirePackage[tight]{subfigure}
\RequirePackage[font=small,labelfont=bf]{caption}
\RequirePackage[noend]{algorithmic}
\RequirePackage{algorithm}
\renewcommand{\algorithmicrequire}{\textbf{Input:}}
\renewcommand{\algorithmicensure}{\textbf{Output:}}
\RequirePackage[margin=1in]{geometry}
\graphicspath{{fig/}}
\definecolor{lightblue}{rgb}{.9,.95,1}
\definecolor{darkblue}{rgb}{0,.2,.39}
\RequirePackage[colorlinks=true, linkcolor=darkblue, anchorcolor=darkblue, citecolor=black, urlcolor=darkblue]{hyperref}
\urlstyle{rm}

\global\let\@course\@empty
\def\course#1{\gdef\@course{#1}}
\course{\huge Algorithmic Robotics \\
  \huge COMP/ELEC/MECH 450/550}

\def\@maketitle{%
  \newpage
  \null
    {\raggedright\sffamily\bfseries\Large\color{darkblue}\@course\\ \huge\@title\\[4pt] \normalsize\mdseries\@author \par}%
  \vskip 2.5em}

\DeclareMathAlphabet{\mathcal}{OMS}{cmsy}{m}{n}

\titleformat{\section}{\sffamily\bfseries\large\color{darkblue}}{Project~\thesection:}{0.5em}{}
\titleformat{\subsection}[block]{\sffamily\bfseries\color{darkblue}}{%
    \makebox[0pt][r]{\thesubsection\hspace{0.5em}}}{0em}{}
\titlespacing{\subsection}{0pt}{*3}{-1em}
\titlespacing{\subsubsection}{0pt}{0pt}{0pt}
\titlespacing*{\paragraph}{0pt}{*1}{.75em}
\parindent=0pt
\parskip=\baselineskip

\definecolor{grey}{rgb}{0.2,0.2,0.2}
\definecolor{mauve}{rgb}{0.58,0,0.82}

\lstset{ %
  backgroundcolor=\color{lightblue},
  basicstyle=\ttfamily\footnotesize,
  breakatwhitespace=false,
  breaklines=true,
  %% captionpos=b,
  %% columns=flexible,
  commentstyle=\color{darkblue},
  escapeinside={\%*}{*)},
  extendedchars=true,
  fontadjust=true,
  frame=single,
  keepspaces=true,
  keywordstyle=\color{blue},
  numbers=left,
  numbersep=5pt,
  numberstyle=\tiny\color{grey},
  rulecolor=\color{black},
  showspaces=false,
  showstringspaces=false,
  showtabs=false,
  stepnumber=1,
  stringstyle=\color{mauve},
  tabsize=2,
  title=\lstname
}

% define a new conditional that we can use to selectively enable/disable content
\newif\ifrice
% uncomment this line when using this document at Rice
\ricetrue

\newcommand\abbr[1]{\textsc{\lowercase{#1}}}
\newcommand\ompl{\textsc{ompl}\xspace}
\newcommand\omplapp{\textsc{ompl}.app\xspace}
\newcommand\fixme[1]{\textcolor{red}{\textbf{#1}}}
\newcommand\bluebox[1]{\vspace*{.5\baselineskip}\centerline{\fboxsep=10pt\fcolorbox{darkblue}{lightblue}{\textbf{\sffamily\color{darkblue}#1}}}}
\newcommand\ompldocinfo{\bluebox{The documentation for \ompl can be found at \urlstyle{sf}\url{http://ompl.kavrakilab.org}.}}
\newcommand{\cspace}{{$\mathcal{C}$-space}\xspace}
