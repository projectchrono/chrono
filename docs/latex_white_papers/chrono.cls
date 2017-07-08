%%
%% `chrono.cls',
%% a LaTeX document class for generating Chrono 
%% technical reports.
%%

\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{chrono}[2016/03/03, 1.0.0]

\ExecuteOptions{a4paper,10pt,onecolumn}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions

\LoadClass{article}
%\RequirePackage[a4paper,top=3cm,bottom=3cm,left=3cm,right=3cm]{geometry}
\RequirePackage{xcolor}
\RequirePackage{titlesec}
\RequirePackage{tikz}
\RequirePackage{tikzpagenodes} 
\RequirePackage{fancyhdr}
\RequirePackage{hyperref}
\RequirePackage{xstring}

\newcommand{\ToDoxygenPage}[1]{%
    \StrSubstitute{#1}{_}{__}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{A}{_a}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{B}{_b}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{C}{_c}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{D}{_d}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{E}{_e}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{F}{_f}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{G}{_g}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{H}{_h}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{I}{_i}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{J}{_j}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{K}{_k}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{L}{_l}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{M}{_m}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{N}{_n}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{O}{_o}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{P}{_p}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{Q}{_q}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{R}{_r}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{S}{_s}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{T}{_t}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{U}{_u}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{V}{_v}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{W}{_w}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{Y}{_y}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{Z}{_z}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{:}{_1}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{/}{_2}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{<}{_3}[\tmpcapit]%
		\StrSubstitute{\tmpcapit}{>}{_4}[\tmpcapit]%
		%\StrSubstitute{\tmpcapit}{.}{.}[\tmpcapit]%
		}

\newcommand{\urlChronoAPIclass}[1]{%
  \ToDoxygenPage{#1}
  \href{http://api.chrono.projectchrono.org/class\tmpcapit.html}{\bfseries\texttt{#1}}%
	}
\newcommand{\urlChronoAPInamespace}[1]{%
  \ToDoxygenPage{#1}
  \href{http://api.chrono.projectchrono.org/namespace\tmpcapit.html}{\bfseries\texttt{#1}}%
	}
\newcommand{\urlChronoAPImodule}[1]{%
  \lowercase{\def\tmpcapit{#1}}%
  \href{http://api.chrono.projectchrono.org/group__\tmpcapit__module.html}{\bfseries\texttt{#1 module}}%
	}

\newcommand{\Chrono}{%
  {\sffamily{\bfseries Chrono}}\space%
	}
\newcommand{\urlChrono}{%
  \href{http://www.projectchrono.org}{\Chrono}%
	}
	
\makeatletter
  \def\@maketitle{%
  \newpage
  \null
	\begin{tikzpicture}[remember picture, overlay]
	\node[inner sep=0pt,anchor=east] at ([yshift=-0.4cm]current page text area.north east)
     {\includegraphics[height=2cm]{logo_chrono_engine_h200}};
  \end{tikzpicture}
  \vskip 4em%
  \let \footnote \thanks
    {\noindent \LARGE \bfseries\sffamily\color{chronosectioncolor} \@title \par}%
    \vskip 1.5em%
    {\sffamily \noindent 
        \@author
		}%
    \vskip 1em%
    {\sffamily \footnotesize \noindent \@date}%
  \par
  \vskip 1.5em
		\thispagestyle{fancy}
		}
\makeatother


\renewenvironment{abstract}
 {{\noindent\textcolor[RGB]{220,220,220}{\rule{\linewidth}{0.2pt}}}
  \small
  \noindent{\sffamily\bfseries\abstractname\vspace{0.5em}\vspace{0pt}}
	}
 {{\noindent\textcolor[RGB]{220,220,220}{\rule{\linewidth}{0.2pt}}}}


\definecolor{chronosectioncolor}{rgb}{0.2 0.35 0.5}

\titleformat{\section}[block]
  {\fontsize{14}{17}\bfseries\sffamily\color{chronosectioncolor}}
  {\thesection.\;}
  {1em}
  {}
\titleformat{\chapter}[display]
  {\normalfont\sffamily\huge\bfseries\color{chronosectioncolor}}
  {\chaptertitlename\ \thechapter}{20pt}{\Huge}

\pagestyle{fancy}
\fancyhead{}
\fancyfoot[C]{\thepage}
\fancyhead[L]{\fontfamily{phv}\selectfont {\footnotesize Chrono technical documentation}}

\hypersetup{
    bookmarks=true,         % show bookmarks bar?
    pdftoolbar=true,        % show Acrobat’s toolbar?
    pdfmenubar=true,        % show Acrobat’s menu?
    pdfauthor={ProjectChrono team},     % author
    pdfsubject={Chrono theory reference},   % subject of the document
    colorlinks=true,       % false: boxed links; true: colored links
    linkcolor=blue,          % color of internal links (change box color with linkbordercolor)
    citecolor=blue,        % color of links to bibliography
    filecolor=blue,      % color of file links
    urlcolor=[rgb]{0,0.3,0.6}           % color of external links
}
